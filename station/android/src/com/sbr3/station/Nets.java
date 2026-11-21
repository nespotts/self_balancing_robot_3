package com.sbr3.station;

import android.content.Context;
import android.net.ConnectivityManager;
import android.net.LinkAddress;
import android.net.LinkProperties;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.NetworkRequest;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.List;

/**
 * The phone's Wi-Fi / Ethernet network: which one to scan, binding the process to it when it isn't the default
 * (a robot hotspot without internet), and undoing that binding when it goes away. Watch tells the app when it changes.
 */
final class Nets {
    private static final String TAG = "BalanceBot";
    private static Network bound;   // what we bound the process to (main-thread and finder-thread writes: synchronized)

    private Nets() {}

    private static ConnectivityManager cm(Context c) {
        return (ConnectivityManager) c.getApplicationContext().getSystemService(Context.CONNECTIVITY_SERVICE);
    }

    /** {ip, prefix} of the Wi-Fi (or Ethernet) IPv4 address to scan, or null (and unbound). */
    static synchronized String[] choose(Context ctx, String lastHost) {
        ConnectivityManager cm = cm(ctx);
        if (cm == null) return null;
        try {
            Network active = cm.getActiveNetwork();
            List<NetPick.Cand> cands = new ArrayList<>();
            for (Network n : cm.getAllNetworks()) {
                NetworkCapabilities caps = cm.getNetworkCapabilities(n);
                if (caps == null || !(caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI)
                    || caps.hasTransport(NetworkCapabilities.TRANSPORT_ETHERNET))) continue;
                if (caps.hasTransport(NetworkCapabilities.TRANSPORT_VPN)
                    || !caps.hasCapability(NetworkCapabilities.NET_CAPABILITY_NOT_VPN)) continue;
                LinkProperties lp = cm.getLinkProperties(n);
                if (lp == null) continue;
                for (LinkAddress la : lp.getLinkAddresses()) {
                    InetAddress a = la.getAddress();
                    if (a instanceof Inet4Address && !a.isLoopbackAddress()) {
                        cands.add(new NetPick.Cand(n, a.getHostAddress(), la.getPrefixLength(), n.equals(active)));
                        break;
                    }
                }
            }
            NetPick.Cand c = NetPick.pick(cands, lastHost);
            if (c == null) {
                unbindLocked(cm);
                return null;
            }
            Network n = (Network) c.handle;
            if (c.isDefault) unbindLocked(cm);                      // Wi-Fi is the default again: stop forcing it
            else if (!n.equals(bound)) { cm.bindProcessToNetwork(n); bound = n; }   // Wi-Fi without internet (robot hotspot)
            return new String[]{c.ip, String.valueOf(c.prefix)};
        } catch (Exception e) {
            Log.w(TAG, "nets: " + e);
            return null;
        }
    }

    /** Give the process back to the system's default network (if we had bound it). */
    static synchronized void unbind(Context ctx) {
        ConnectivityManager cm = cm(ctx);
        if (cm != null) unbindLocked(cm);
    }

    private static void unbindLocked(ConnectivityManager cm) {
        if (bound == null) return;
        try { cm.bindProcessToNetwork(null); } catch (Exception e) { Log.w(TAG, "unbind: " + e); }
        bound = null;
    }

    // ------------------------------------------------------------------ watching

    interface Listener { void onChanged(boolean up, String ip, String subnet); }

    /** Wi-Fi / Ethernet callbacks while the activity is started; reports a change (debounced) once the choice differs. */
    static final class Watch {
        private final Context ctx;
        private final Listener listener;
        private final Handler h = new Handler(Looper.getMainLooper());
        private ConnectivityManager.NetworkCallback cb;
        private String sig = "";
        private String lastHost;

        Watch(Context c, Listener l) { ctx = c.getApplicationContext(); listener = l; }

        private final Runnable check = new Runnable() {
            @Override public void run() {
                String[] w = choose(ctx, lastHost);
                String s = w == null ? "" : w[0] + "/" + w[1];
                if (s.equals(sig)) return;
                sig = s;
                listener.onChanged(w != null, w == null ? "" : w[0], w == null ? "" : Lan.subnetLabel(w[0]));
            }
        };

        void start(String lastHostNow) {
            stop();
            lastHost = lastHostNow;
            String[] w = choose(ctx, lastHost);            // the starting point: no event for it
            sig = w == null ? "" : w[0] + "/" + w[1];
            ConnectivityManager cm = cm(ctx);
            if (cm == null) return;
            cb = new ConnectivityManager.NetworkCallback() {
                @Override public void onAvailable(Network n) { poke(); }
                @Override public void onLost(Network n) { poke(); }
                @Override public void onLinkPropertiesChanged(Network n, LinkProperties lp) { poke(); }
            };
            try {
                NetworkRequest req = new NetworkRequest.Builder()
                    .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
                    .addTransportType(NetworkCapabilities.TRANSPORT_ETHERNET).build();
                cm.registerNetworkCallback(req, cb, h);
            } catch (Exception e) {
                Log.w(TAG, "net callback: " + e);
                cb = null;
            }
        }

        void setLastHost(String host) { lastHost = host; }

        private void poke() {
            h.removeCallbacks(check);
            h.postDelayed(check, 500);
        }

        void stop() {
            h.removeCallbacks(check);
            ConnectivityManager cm = cm(ctx);
            if (cb != null && cm != null) {
                try { cm.unregisterNetworkCallback(cb); } catch (Exception ignored) { }
            }
            cb = null;
        }
    }
}
