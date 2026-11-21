package com.sbr3.station;

import java.util.List;

/** Pure (desktop-JVM tested): which of the phone's Wi-Fi / Ethernet networks to scan and bind to. */
final class NetPick {
    private NetPick() {}

    /** One candidate network: `handle` is the platform Network (opaque here). */
    static final class Cand {
        final Object handle;
        final String ip;
        final int prefix;
        final boolean isDefault;
        Cand(Object handle, String ip, int prefix, boolean isDefault) {
            this.handle = handle; this.ip = ip; this.prefix = prefix; this.isDefault = isDefault;
        }
    }

    /** Is `host` inside ip/prefix? */
    static boolean sameSubnet(String ip, int prefix, String host) {
        int[] a = Lan.ipv4(ip), b = Lan.ipv4(host);
        if (a == null || b == null || prefix < 0 || prefix > 32) return false;
        long x = ((long) a[0] << 24) | (a[1] << 16) | (a[2] << 8) | a[3];
        long y = ((long) b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3];
        long mask = prefix == 0 ? 0 : (0xFFFFFFFFL << (32 - prefix)) & 0xFFFFFFFFL;
        return (x & mask) == (y & mask);
    }

    /** The network holding the last robot's address, else the default network, else the first. null = none. */
    static Cand pick(List<Cand> cs, String lastHost) {
        if (cs == null || cs.isEmpty()) return null;
        if (lastHost != null) for (Cand c : cs) if (sameSubnet(c.ip, c.prefix, lastHost)) return c;
        for (Cand c : cs) if (c.isDefault) return c;
        return cs.get(0);
    }
}
