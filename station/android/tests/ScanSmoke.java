package com.sbr3.station;

import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/** Live LAN check of the finder's probe + scan (same timeouts, 48 threads), from a PC. Only GETs /api/status. */
public class ScanSmoke {
    public static void main(String[] args) throws Exception {
        String ip = null;
        for (NetworkInterface ni : Collections.list(NetworkInterface.getNetworkInterfaces())) {
            if (!ni.isUp() || ni.isLoopback() || ni.isVirtual()) continue;
            for (InetAddress a : Collections.list(ni.getInetAddresses()))
                if (a instanceof Inet4Address && Lan.isPrivateIpv4(Lan.ipv4(a.getHostAddress()))
                    && a.getHostAddress().startsWith("192.168.")) ip = a.getHostAddress();
        }
        System.out.println("PC " + ip + ", scanning " + Lan.subnetLabel(ip));
        List<String> hosts = Lan.scanList(ip, 24, null);
        long t0 = System.currentTimeMillis();
        List<Lan.Hit> hits = Collections.synchronizedList(new ArrayList<Lan.Hit>());
        ExecutorService p = Executors.newFixedThreadPool(48);
        for (String h : hosts) {
            p.execute(() -> {
                String body = Finder.fetchStatus(h + ":" + Lan.PORT, 400, 1200);
                if (Lan.looksLikeBridge(body)) {
                    Lan.Hit hit = Lan.Hit.of(h + ":" + Lan.PORT, body);
                    hits.add(hit);
                    System.out.println("found " + hit + " after " + (System.currentTimeMillis() - t0) + " ms");
                }
            });
        }
        p.shutdown();
        p.awaitTermination(60, TimeUnit.SECONDS);
        List<Lan.Hit> ranked = Lan.rank(hits, null);
        Lan.Hit auto = Lan.autoPick(ranked);
        System.out.println("scan of " + hosts.size() + " hosts done in " + (System.currentTimeMillis() - t0) + " ms: "
            + ranked + (auto != null ? ", opens " + auto : ranked.isEmpty() ? "" : ", asks"));
        System.exit(ranked.isEmpty() ? 1 : 0);
    }
}