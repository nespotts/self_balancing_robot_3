package com.sbr3.station;

import android.content.Context;
import android.net.ConnectivityManager;
import android.net.LinkAddress;
import android.net.LinkProperties;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.nsd.NsdManager;
import android.net.nsd.NsdServiceInfo;
import android.util.Log;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.json.JSONObject;

/**
 * Finds the bridge: GET http://host:8080/api/status on the last address, NSD (_http._tcp) hits and every host of
 * the phone's Wi-Fi /24, in parallel with short timeouts; stops at the first answer that looks like the bridge.
 * Events: find {phase, subnet, total, done}, found {addr, status}, findDone {found:false}, probe {addr, ok, error}.
 */
final class Finder {
    private static final String TAG = "BalanceBot";
    private static final int THREADS = 48;
    private final Context ctx;
    private final Object lock = new Object();
    private ExecutorService pool;
    private NsdManager nsd;
    private NsdManager.DiscoveryListener nsdListener;
    private volatile int gen;

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

    /** {ip, prefix} of the Wi-Fi (or Ethernet) IPv4 address, or null. Binds the process to it if it isn't the default. */
    String[] wifiIpv4() {
        ConnectivityManager cm = (ConnectivityManager) ctx.getSystemService(Context.CONNECTIVITY_SERVICE);
        if (cm == null) return null;
        try {
            for (Network n : cm.getAllNetworks()) {
                NetworkCapabilities caps = cm.getNetworkCapabilities(n);
                if (caps == null || !(caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI)
                    || caps.hasTransport(NetworkCapabilities.TRANSPORT_ETHERNET))) continue;
                LinkProperties lp = cm.getLinkProperties(n);
                if (lp == null) continue;
                for (LinkAddress la : lp.getLinkAddresses()) {
                    InetAddress a = la.getAddress();
                    if (a instanceof Inet4Address && !a.isLoopbackAddress()) {
                        // Wi-Fi without internet (e.g. a robot hotspot) isn't the default network: route our sockets there
                        if (!n.equals(cm.getActiveNetwork())) cm.bindProcessToNetwork(n);
                        return new String[]{a.getHostAddress(), String.valueOf(la.getPrefixLength())};
                    }
                }
            }
        } catch (Exception e) {
            Log.w(TAG, "wifiIpv4: " + e);
        }
        return null;
    }

    void start(final String last) {
        stop();
        final int g;
        final ExecutorService p = Executors.newFixedThreadPool(THREADS);
        synchronized (lock) { g = ++gen; pool = p; }
        String[] w = wifiIpv4();
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
        final AtomicBoolean found = new AtomicBoolean(false);
        final AtomicInteger done = new AtomicInteger();
        final String subnet = w == null ? "" : Lan.subnetLabel(w[0]);
        emitFind(g, "scan", subnet, total, 0, w != null);
        if (total == 0) { emitDone(g); return; }
        startNsd(g, p, found);
        for (int i = 0; i < total; i++) {
            final String t = targets.get(i);
            final boolean isLast = t.equals(last);
            p.execute(() -> {
                if (g != gen || found.get()) return;
                String body = isLast ? fetchStatus(t, 1500, 2000) : fetchStatus(t, 400, 1200);
                if (g != gen) return;
                if (Lan.looksLikeBridge(body)) hit(g, t, body, found);
                int d = done.incrementAndGet();
                if (found.get()) return;
                if (d % 12 == 0 || d == total) emitFind(g, "scan", subnet, total, d, true);
                if (d == total) emitDone(g);
            });
        }
    }

    private void hit(int g, String addr, String body, AtomicBoolean found) {
        if (!found.compareAndSet(false, true)) return;
        try {
            Events.emit("found", new JSONObject().put("addr", addr).put("status", new JSONObject(body)));
        } catch (Exception e) {
            Events.emit("found", "{\"addr\":" + JSONObject.quote(addr) + "}");
        }
        stopIf(g);
    }

    private void emitFind(int g, String phase, String subnet, int total, int done, boolean wifi) {
        if (g != gen) return;
        try {
            Events.emit("find", new JSONObject().put("phase", phase).put("subnet", subnet).put("total", total)
                .put("done", done).put("wifi", wifi));
        } catch (Exception ignored) {
        }
    }

    private void emitDone(int g) {
        if (g != gen) return;
        Events.emit("findDone", "{\"found\":false}");
        stopIf(g);
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
                    if (Lan.looksLikeBridge(body)) o.put("ok", true).put("status", new JSONObject(body));
                    else o.put("ok", false).put("error", body == null ? "No answer from " + addr : "Not the robot's bridge");
                }
            } catch (Exception e) {
                try { o.put("ok", false).put("error", String.valueOf(e.getMessage())); } catch (Exception ignored) { }
            }
            Events.emit("probe", o);
        }, "probe").start();
    }

    // ------------------------------------------------------------------ NSD (_http._tcp), best effort

    private void startNsd(final int g, final ExecutorService p, final AtomicBoolean found) {
        try {
            nsd = (NsdManager) ctx.getSystemService(Context.NSD_SERVICE);
            if (nsd == null) return;
            final ExecutorService resolver = Executors.newSingleThreadExecutor();
            nsdListener = new NsdManager.DiscoveryListener() {
                @Override public void onDiscoveryStarted(String t) { }
                @Override public void onDiscoveryStopped(String t) { resolver.shutdownNow(); }
                @Override public void onStartDiscoveryFailed(String t, int e) { resolver.shutdownNow(); }
                @Override public void onStopDiscoveryFailed(String t, int e) { }
                @Override public void onServiceLost(NsdServiceInfo s) { }
                @Override public void onServiceFound(final NsdServiceInfo s) {
                    try { resolver.execute(() -> resolve(g, p, found, s)); } catch (Exception ignored) { }
                }
            };
            nsd.discoverServices("_http._tcp", NsdManager.PROTOCOL_DNS_SD, nsdListener);
        } catch (Exception e) {
            Log.w(TAG, "nsd: " + e);
            nsdListener = null;
        }
    }

    @SuppressWarnings("deprecation")
    private void resolve(final int g, final ExecutorService p, final AtomicBoolean found, NsdServiceInfo s) {
        if (g != gen || found.get() || nsd == null) return;
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
                        try {
                            p.execute(() -> {
                                if (g != gen || found.get()) return;
                                String body = fetchStatus(hp, 800, 1500);
                                if (g == gen && Lan.looksLikeBridge(body)) hit(g, hp, body, found);
                            });
                        } catch (Exception ignored) { }
                    }
                }
                latch.countDown();
            }
        });
        try { latch.await(3, TimeUnit.SECONDS); } catch (InterruptedException ignored) { }
    }

    private void stopNsd() {
        NsdManager n = nsd;
        NsdManager.DiscoveryListener l = nsdListener;
        nsdListener = null;
        if (n != null && l != null) {
            try { n.stopServiceDiscovery(l); } catch (Exception ignored) { }
        }
    }
}
