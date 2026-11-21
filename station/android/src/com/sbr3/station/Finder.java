package com.sbr3.station;

import android.content.Context;
import android.net.ConnectivityManager;
import android.net.LinkAddress;
import android.net.LinkProperties;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.nsd.NsdManager;
import android.net.wifi.WifiManager;
import android.net.nsd.NsdServiceInfo;
import android.util.Log;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.json.JSONArray;
import org.json.JSONObject;

/**
 * Finds the bridge: GET http://host:8080/api/status on the last address, NSD (_http._tcp) hits and every host of
 * the phone's Wi-Fi /24, in parallel with short timeouts (~2 s). Always finishes the scan and reports every bridge,
 * each a real robot (serial /dev/...) or a simulator (tcp://...), so a sim on the same Wi-Fi can't win by answering first.
 * Events: find {phase, subnet, total, done}, findDone {hits: [{addr, kind, alive}], auto}, probe {addr, ok, kind, error}.
 */
final class Finder {
    private static final String TAG = "BalanceBot";
    private static final int THREADS = 48;
    private final Context ctx;
    private final Object lock = new Object();
    private ExecutorService pool;
    private NsdManager nsd;
    private NsdManager.DiscoveryListener nsdListener;
    private WifiManager.MulticastLock mcast;   // held while NSD runs: some phones drop mDNS packets otherwise
    private volatile int gen;
    private final ConcurrentHashMap<String, String> kinds = new ConcurrentHashMap<>();   // addr -> robot / sim

    Finder(Context c) { ctx = c.getApplicationContext(); }

    /** Read /api/status; the body if it answered 200, else null. Never throws. */
    static String fetchStatus(String hostPort, int connectMs, int readMs) {
        HttpURLConnection c = null;
        try {
            c = (HttpURLConnection) new URL("http://" + hostPort + "/api/status").openConnection();
            c.setConnectTimeout(connectMs);
            c.setReadTimeout(readMs);
            c.setUseCaches(false);
            c.setInstanceFollowRedirects(false);
            if (c.getResponseCode() != 200) return null;
            try (InputStream in = c.getInputStream()) {
                ByteArrayOutputStream out = new ByteArrayOutputStream();
                byte[] buf = new byte[2048];
                int n;
                while ((n = in.read(buf)) > 0 && out.size() < 16384) out.write(buf, 0, n);
                return out.toString("UTF-8");
            }
        } catch (Exception e) {
            return null;
        } finally {
            if (c != null) c.disconnect();
        }
    }

    /** {ip, prefix} of the Wi-Fi (or Ethernet) IPv4 address to scan, or null. Binds / unbinds the process (see Nets). */
    String[] wifiIpv4(String lastHost) { return Nets.choose(ctx, lastHost); }
    void start(final String last) {
        stop();
        final int g;
        final ExecutorService p = Executors.newFixedThreadPool(THREADS);
        synchronized (lock) { g = ++gen; pool = p; }
        String[] w = wifiIpv4(last == null ? null : Lan.hostOf(last));
        final List<String> targets = new ArrayList<>();
        if (last != null) targets.add(last);
        String lastHost = last == null ? null : Lan.hostOf(last);
        if (w != null) {
            for (String ip : Lan.scanList(w[0], Integer.parseInt(w[1]), lastHost)) {
                String hp = ip + ":" + Lan.PORT;
                if (!hp.equals(last)) targets.add(hp);
            }
        }
        final int total = targets.size();
        final Scan s = new Scan(g, p, last);
        final String subnet = w == null ? "" : Lan.subnetLabel(w[0]);
        emitFind(g, "scan", subnet, total, 0, w != null);
        if (total == 0) { finish(s); return; }
        startNsd(s);
        for (int i = 0; i < total; i++) {
            final String t = targets.get(i);
            final boolean isLast = t.equals(last);
            s.seen.add(t);
            boolean queued = s.submit(() -> {
                String body = isLast ? fetchStatus(t, 1500, 2000) : fetchStatus(t, 400, 1200);
                if (g != gen) return;
                if (Lan.looksLikeBridge(body)) s.hit(t, body);
                int d = s.done.incrementAndGet();
                if (d % 12 == 0 || d == total) emitFind(g, "scan", subnet, total, d, true);
            });
            if (!queued) return;   // stopped meanwhile
        }
        s.release();   // all scan probes queued: the scan ends when the last probe (or NSD probe) returns
    }

    /** One run of start(): every address probed, every bridge that answered. Ends when no probe is pending. */
    private final class Scan {
        final int g;
        final ExecutorService p;
        final String last;
        final Set<String> seen = Collections.newSetFromMap(new ConcurrentHashMap<String, Boolean>());
        final Map<String, Lan.Hit> hits = new ConcurrentHashMap<>();
        final AtomicInteger done = new AtomicInteger();
        final AtomicInteger pending = new AtomicInteger(1);   // the 1 = start() still queueing
        final AtomicBoolean finished = new AtomicBoolean();

        Scan(int g, ExecutorService p, String last) { this.g = g; this.p = p; this.last = last; }

        boolean submit(final Runnable r) {
            if (g != gen) return false;
            pending.incrementAndGet();
            try {
                p.execute(() -> {
                    try { if (g == gen) r.run(); } finally { release(); }
                });
                return true;
            } catch (Exception e) {   // pool shut down: stopped
                pending.decrementAndGet();
                return false;
            }
        }

        void release() { if (pending.decrementAndGet() == 0) finish(this); }

        void hit(String addr, String body) {
            Lan.Hit h = Lan.Hit.of(addr, body);
            hits.put(addr, h);
            kinds.put(addr, h.kind);
        }
    }

    /** findDone {hits: [{addr, kind, alive}] robots first, auto: the only robot's addr or null}. */
    private void finish(Scan s) {
        if (s.g != gen || !s.finished.compareAndSet(false, true)) return;
        List<Lan.Hit> ranked = Lan.rank(new ArrayList<>(s.hits.values()), s.last);
        Lan.Hit auto = Lan.autoPick(ranked);
        try {
            JSONArray a = new JSONArray();
            for (Lan.Hit h : ranked) a.put(new JSONObject().put("addr", h.addr).put("kind", h.kind).put("alive", h.alive));
            Events.emit("findDone", new JSONObject().put("hits", a).put("auto", auto == null ? JSONObject.NULL : auto.addr));
        } catch (Exception e) {
            Events.emit("findDone", "{\"hits\":[]}");
        }
        stopIf(s.g);
    }

    /** "robot" / "sim" for an address the scan or a probe has seen, else null. */
    String kindOf(String addr) { return addr == null ? null : kinds.get(addr); }
    private void emitFind(int g, String phase, String subnet, int total, int done, boolean wifi) {
        if (g != gen) return;
        try {
            Events.emit("find", new JSONObject().put("phase", phase).put("subnet", subnet).put("total", total)
                .put("done", done).put("wifi", wifi));
        } catch (Exception ignored) {
        }
    }

    private void stopIf(int g) { if (g == gen) stop(); }

    void stop() {
        ExecutorService p;
        synchronized (lock) { gen++; p = pool; pool = null; }
        if (p != null) p.shutdownNow();
        stopNsd();
    }

    /** Manual address: one probe with longer timeouts -> probe {addr, ok, error}. */
    void probe(final String raw) {
        new Thread(() -> {
            String addr = Lan.normalize(raw);
            JSONObject o = new JSONObject();
            try {
                o.put("addr", addr == null ? raw : addr);
                if (addr == null) o.put("ok", false).put("error", "Not an address (e.g. 192.168.0.209)");
                else if (!Lan.isPrivateHost(Lan.hostOf(addr))) o.put("ok", false).put("error", "Only local network addresses");
                else {
                    String body = fetchStatus(addr, 2500, 3000);
                    if (Lan.looksLikeBridge(body)) {
                        String k = Lan.kindOf(body);
                        kinds.put(addr, k);
                        o.put("ok", true).put("kind", k).put("alive", Lan.robotAlive(body));
                    }
                    else o.put("ok", false).put("error", body == null ? "No answer from " + addr : "Not the robot's bridge");
                }
            } catch (Exception e) {
                try { o.put("ok", false).put("error", String.valueOf(e.getMessage())); } catch (Exception ignored) { }
            }
            Events.emit("probe", o);
        }, "probe").start();
    }

    // ------------------------------------------------------------------ NSD (_http._tcp), best effort

    private void startNsd(final Scan sc) {
        try {
            nsd = (NsdManager) ctx.getSystemService(Context.NSD_SERVICE);
            if (nsd == null) return;
            lockMulticast();
            final ExecutorService resolver = Executors.newSingleThreadExecutor();
            nsdListener = new NsdManager.DiscoveryListener() {
                @Override public void onDiscoveryStarted(String t) { }
                @Override public void onDiscoveryStopped(String t) { resolver.shutdownNow(); }
                @Override public void onStartDiscoveryFailed(String t, int e) { resolver.shutdownNow(); }
                @Override public void onStopDiscoveryFailed(String t, int e) { }
                @Override public void onServiceLost(NsdServiceInfo s) { }
                @Override public void onServiceFound(final NsdServiceInfo s) {
                    try { resolver.execute(() -> resolve(sc, s)); } catch (Exception ignored) { }
                }
            };
            nsd.discoverServices("_http._tcp", NsdManager.PROTOCOL_DNS_SD, nsdListener);
        } catch (Exception e) {
            Log.w(TAG, "nsd: " + e);
            nsdListener = null;
        }
    }

    @SuppressWarnings("deprecation")
    private void resolve(final Scan sc, NsdServiceInfo s) {
        if (sc.g != gen || nsd == null) return;
        final CountDownLatch latch = new CountDownLatch(1);
        nsd.resolveService(s, new NsdManager.ResolveListener() {
            @Override public void onResolveFailed(NsdServiceInfo si, int e) { latch.countDown(); }
            @Override public void onServiceResolved(NsdServiceInfo si) {
                InetAddress h = si.getHost();
                if (h instanceof Inet4Address && Lan.isPrivateIpv4(Lan.ipv4(h.getHostAddress()))) {
                    final List<String> hps = new ArrayList<>();
                    hps.add(h.getHostAddress() + ":" + si.getPort());
                    if (si.getPort() != Lan.PORT) hps.add(h.getHostAddress() + ":" + Lan.PORT);
                    for (final String hp : hps) {
                        if (!sc.seen.add(hp)) continue;   // the /24 scan already asks it
                        sc.submit(() -> {
                            String body = fetchStatus(hp, 800, 1500);
                            if (sc.g == gen && Lan.looksLikeBridge(body)) sc.hit(hp, body);
                        });
                    }
                }
                latch.countDown();
            }
        });
        try { latch.await(3, TimeUnit.SECONDS); } catch (InterruptedException ignored) { }
    }

    private synchronized void lockMulticast() {
        try {
            WifiManager wm = (WifiManager) ctx.getSystemService(Context.WIFI_SERVICE);
            if (wm == null || mcast != null) return;
            mcast = wm.createMulticastLock("sbr3-nsd");
            mcast.setReferenceCounted(false);
            mcast.acquire();
        } catch (Exception e) {
            Log.w(TAG, "multicast lock: " + e);
            mcast = null;
        }
    }

    private synchronized void unlockMulticast() {
        WifiManager.MulticastLock l = mcast;
        mcast = null;
        if (l != null) {
            try { if (l.isHeld()) l.release(); } catch (Exception ignored) { }
        }
    }

    private void stopNsd() {
        unlockMulticast();
        NsdManager n = nsd;
        NsdManager.DiscoveryListener l = nsdListener;
        nsdListener = null;
        if (n != null && l != null) {
            try { n.stopServiceDiscovery(l); } catch (Exception ignored) { }
        }
    }
}
