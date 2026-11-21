package com.sbr3.station;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.regex.Pattern;

/** Pure helpers (no Android imports, unit-tested on the desktop JVM): addresses, private-LAN checks, scan order. */
final class Lan {
    static final int PORT = 8080;
    private static final Pattern HOST_PORT = Pattern.compile("[A-Za-z0-9.-]{1,253}(:[0-9]{1,5})?");
    private static final Pattern STATUS_T = Pattern.compile("\"t\"\\s*:\\s*\"status\"");
    private static final Pattern STATUS_LINK = Pattern.compile("\"link\"\\s*:");

    private Lan() {}

    /** "http://192.168.0.209:8080/x" / " 192.168.0.209 " -> "192.168.0.209:8080"; null if it isn't a plain host[:port]. */
    static String normalize(String addr) {
        if (addr == null) return null;
        String a = addr.trim();
        int s = a.indexOf("://");
        if (s >= 0) a = a.substring(s + 3);
        int slash = a.indexOf('/');
        if (slash >= 0) a = a.substring(0, slash);
        if (a.isEmpty() || !HOST_PORT.matcher(a).matches()) return null;
        int c = a.indexOf(':');
        if (c < 0) return a.toLowerCase(Locale.ROOT) + ":" + PORT;
        int port = Integer.parseInt(a.substring(c + 1));
        if (port < 1 || port > 65535 || c == 0) return null;
        return a.substring(0, c).toLowerCase(Locale.ROOT) + ":" + port;
    }

    static String hostOf(String hostPort) {
        int c = hostPort.lastIndexOf(':');
        return c < 0 ? hostPort : hostPort.substring(0, c);
    }

    /** Dotted IPv4 literal -> 4 bytes (0..255), or null. */
    static int[] ipv4(String host) {
        if (host == null) return null;
        String[] p = host.split("\\.", -1);
        if (p.length != 4) return null;
        int[] b = new int[4];
        for (int i = 0; i < 4; i++) {
            if (p[i].isEmpty() || p[i].length() > 3) return null;
            for (int k = 0; k < p[i].length(); k++) if (!Character.isDigit(p[i].charAt(k))) return null;
            b[i] = Integer.parseInt(p[i]);
            if (b[i] > 255) return null;
        }
        return b;
    }

    /** RFC 1918, loopback, link-local (169.254/16) IPv4. */
    static boolean isPrivateIpv4(int[] b) {
        if (b == null) return false;
        return b[0] == 10 || b[0] == 127 || (b[0] == 172 && b[1] >= 16 && b[1] <= 31)
            || (b[0] == 192 && b[1] == 168) || (b[0] == 169 && b[1] == 254);
    }

    /** Names that only resolve on the local network: localhost, mDNS (.local), single-label, .lan/.home.arpa/.internal. */
    static boolean isLocalName(String host) {
        if (host == null || host.isEmpty()) return false;
        String h = host.toLowerCase(Locale.ROOT);
        if (h.endsWith(".")) h = h.substring(0, h.length() - 1);
        return h.equals("localhost") || h.indexOf('.') < 0 || h.endsWith(".local") || h.endsWith(".lan")
            || h.endsWith(".home.arpa") || h.endsWith(".internal");
    }

    /** May the app talk plain http to this host without a DNS lookup? (IP literal in a private range, or a local name.) */
    static boolean isPrivateHost(String host) {
        int[] b = ipv4(host);
        if (b != null) return isPrivateIpv4(b);
        return isLocalName(host);
    }

    /** Does an /api/status body look like the SBR3 bridge? ({"t": "status", "link": ..., ...}) */
    static boolean looksLikeBridge(String body) {
        return body != null && body.trim().startsWith("{") && STATUS_T.matcher(body).find() && STATUS_LINK.matcher(body).find();
    }

    /**
     * Hosts to probe on the phone's subnet: at most the /24 around phoneIp (or the smaller subnet if prefix > 24),
     * without the phone itself and the network/broadcast addresses. `first` (e.g. the last robot's IP) goes first.
     */
    static List<String> scanList(String phoneIp, int prefix, String first) {
        List<String> out = new ArrayList<>();
        int[] b = ipv4(phoneIp);
        if (b == null) return out;
        int p = Math.max(24, Math.min(30, prefix <= 0 ? 24 : prefix));
        int size = 1 << (32 - p);
        int base = b[3] & ~(size - 1);
        String net = b[0] + "." + b[1] + "." + b[2] + ".";
        int[] f = ipv4(first);
        if (f != null && f[0] == b[0] && f[1] == b[1] && f[2] == b[2] && f[3] != b[3]
            && f[3] > base && f[3] < base + size - 1) out.add(first);
        for (int i = base + 1; i < base + size - 1; i++) {
            String ip = net + i;
            if (i != b[3] && !ip.equals(first)) out.add(ip);
        }
        return out;
    }

    /** "192.168.0.x" label of the scanned range, for the find screen. */
    static String subnetLabel(String phoneIp) {
        int[] b = ipv4(phoneIp);
        return b == null ? "" : b[0] + "." + b[1] + "." + b[2] + ".x";
    }
}
