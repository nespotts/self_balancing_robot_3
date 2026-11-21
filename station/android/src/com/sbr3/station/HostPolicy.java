package com.sbr3.station;

import java.net.Inet4Address;
import java.net.Inet6Address;
import java.net.InetAddress;
import java.net.URI;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * Pure (no Android imports, unit-tested on the desktop JVM): which pages and hosts the app trusts.
 * Cleartext is allowed app-wide (network_security_config), so these checks are the only guard:
 *   - a private IPv4 literal is trusted as is;
 *   - a NAME is trusted only if it resolved to private addresses only, and then it is pinned: a later answer with an
 *     address that was not in the pin (DNS rebinding) is refused until the network changes (clear()).
 * URLs are parsed with java.net.URI, strictly: userinfo ("10.0.0.1@evil.com"), backslashes, IPv6 literals -> no host.
 */
final class HostPolicy {
    static final String FINDER = "file:///android_asset/find.html";

    interface Resolver { List<InetAddress> resolve(String host) throws Exception; }

    private static final class Pin {
        final Set<String> addrs = new HashSet<>();
        boolean ok;
        boolean rebound;
        long at;
    }

    private final Resolver resolver;
    private final long ttlMs;
    private final Map<String, Pin> pins = new HashMap<>();
    private long fakeNow = -1;

    HostPolicy(Resolver r, long ttlMs) { resolver = r; this.ttlMs = ttlMs; }

    /** tests only: a clock; -1 = real time. */
    void setNow(long t) { fakeNow = t; }

    private long now() { return fakeNow >= 0 ? fakeNow : System.currentTimeMillis(); }

    // ------------------------------------------------------------------ URLs

    /** The bundled find screen (exactly the asset, optionally with #hash / ?query). */
    static boolean isFinder(String url) {
        if (url == null || !url.startsWith(FINDER)) return false;
        return url.length() == FINDER.length() || url.charAt(FINDER.length()) == '#' || url.charAt(FINDER.length()) == '?';
    }

    /** Lower-case host of a plain http URL without userinfo, else null (https, intent:, file:, junk). */
    static String httpHost(String url) {
        URI u = parse(url);
        if (u == null || !"http".equals(scheme(u))) return null;
        return hostOf(u);
    }

    /** "host:port" (port 80 when none) of an http URL, else null. */
    static String hostPort(String url) {
        URI u = parse(url);
        if (u == null || !"http".equals(scheme(u))) return null;
        String h = hostOf(u);
        return h == null ? null : h + ":" + (u.getPort() < 0 ? 80 : u.getPort());
    }

    /** Lower-case scheme of any URL, or "" when it doesn't parse. */
    static String schemeOf(String url) {
        URI u = parse(url);
        return u == null ? "" : scheme(u);
    }

    private static URI parse(String url) {
        if (url == null) return null;
        try {
            return new URI(url);
        } catch (Exception e) {
            return null;
        }
    }

    private static String scheme(URI u) { return u.getScheme() == null ? "" : u.getScheme().toLowerCase(Locale.ROOT); }

    private static String hostOf(URI u) {
        if (u.getUserInfo() != null) return null;
        String h = u.getHost();
        if (h == null || h.isEmpty() || h.indexOf('[') >= 0 || h.indexOf('%') >= 0) return null;
        if (h.endsWith(".")) h = h.substring(0, h.length() - 1);
        return h.isEmpty() ? null : h.toLowerCase(Locale.ROOT);
    }

    // ------------------------------------------------------------------ addresses

    static boolean isPrivateAddr(InetAddress a) {
        if (a instanceof Inet4Address) {
            byte[] b = a.getAddress();
            return Lan.isPrivateIpv4(new int[]{b[0] & 255, b[1] & 255, b[2] & 255, b[3] & 255});
        }
        if (a instanceof Inet6Address) {
            byte[] b = a.getAddress();
            return a.isLoopbackAddress() || a.isLinkLocalAddress() || (b[0] & 0xfe) == 0xfc;   // ::1, fe80::/10, fc00::/7
        }
        return false;
    }

    // ------------------------------------------------------------------ hosts

    /**
     * May the app talk to this host? Private IPv4 literal: yes. A name: resolve it (blocking: not on the UI thread),
     * every address must be private and none may be new compared with the pin. Cached for ttlMs.
     */
    synchronized boolean allow(String host) {
        if (host == null || host.isEmpty()) return false;
        int[] ip = Lan.ipv4(host);
        if (ip != null) return Lan.isPrivateIpv4(ip);
        Pin p = pins.get(host);
        long t = now();
        if (p != null && (p.rebound || t - p.at < ttlMs)) return p.ok && !p.rebound;
        Set<String> seen;
        boolean allPrivate;
        try {
            List<InetAddress> all = resolver.resolve(host);
            seen = new HashSet<>();
            allPrivate = all != null && !all.isEmpty();
            if (all != null) for (InetAddress a : all) { seen.add(a.getHostAddress()); allPrivate &= isPrivateAddr(a); }
        } catch (Exception e) {
            return false;      // no answer: nothing pinned, ask again next time
        }
        if (p == null) {
            p = new Pin();
            pins.put(host, p);
            p.addrs.addAll(seen);
        } else if (!p.addrs.containsAll(seen)) {
            p.rebound = true;  // an address that was not there when we first looked: refuse until the network changes
            p.ok = false;
            return false;
        }
        p.ok = allPrivate;
        p.at = t;
        return p.ok;
    }

    /** No DNS: is this host already known good (private literal, or a name that was resolved + pinned as private)? */
    synchronized boolean isKnownGood(String host) {
        if (host == null) return false;
        int[] ip = Lan.ipv4(host);
        if (ip != null) return Lan.isPrivateIpv4(ip);
        Pin p = pins.get(host);
        return p != null && p.ok && !p.rebound;
    }

    /** Network changed: forget the pins. */
    synchronized void clear() { pins.clear(); }

    /** Is this page allowed to use SBR3Native? The find screen, a private literal, or a pinned private name. No DNS. */
    boolean trustedPage(String url) {
        if (isFinder(url)) return true;
        return isKnownGood(httpHost(url));
    }
}
