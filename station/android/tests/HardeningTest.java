package com.sbr3.station;

import java.net.InetAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Desktop-JVM tests for the hardening helpers (issues #31, #33, #34): HostPolicy, NetPick, RateLimit, GestureRects, TiltMath. */
public class HardeningTest {
    static int fails, checks;

    static void ok(boolean c, String what) {
        checks++;
        if (!c) { fails++; System.out.println("FAIL " + what); }
    }

    static void eq(Object a, Object b, String what) { ok(a == null ? b == null : a.equals(b), what + ": " + a + " != " + b); }

    static InetAddress ip(String s) throws Exception {
        String[] p = s.split("\\.");
        return InetAddress.getByAddress(new byte[]{(byte) Integer.parseInt(p[0]), (byte) Integer.parseInt(p[1]),
            (byte) Integer.parseInt(p[2]), (byte) Integer.parseInt(p[3])});
    }

    /** A DNS the test controls: host -> addresses (missing = no answer). */
    static final class FakeDns implements HostPolicy.Resolver {
        final Map<String, List<InetAddress>> map = new HashMap<>();
        int lookups;
        void set(String host, String... ips) throws Exception {
            List<InetAddress> l = new ArrayList<>();
            for (String s : ips) l.add(ip(s));
            map.put(host, l);
        }
        @Override public List<InetAddress> resolve(String host) throws Exception {
            lookups++;
            List<InetAddress> l = map.get(host);
            if (l == null) throw new java.net.UnknownHostException(host);
            return l;
        }
    }

    public static void main(String[] args) throws Exception {
        urls();
        names();
        nets();
        limit();
        rects();
        tiltFrames();
        System.out.println((fails == 0 ? "OK " : "FAILED ") + (checks - fails) + "/" + checks + " checks");
        System.exit(fails == 0 ? 0 : 1);
    }

    // ------------------------------------------------------------------ #34: which page gets SBR3Native

    static void urls() throws Exception {
        FakeDns dns = new FakeDns();
        HostPolicy p = new HostPolicy(dns, 10_000);
        ok(p.trustedPage("file:///android_asset/find.html"), "find screen");
        ok(p.trustedPage("file:///android_asset/find.html#error=1&addr=x"), "find screen with hash");
        ok(!p.trustedPage("file:///android_asset/find.html/../../x"), "find screen path trick");
        ok(!p.trustedPage("file:///android_asset/other.html"), "other asset");
        ok(p.trustedPage("http://192.168.0.209:8080/"), "private ip");
        ok(p.trustedPage("http://10.1.2.3/#drive"), "10/8");
        ok(!p.trustedPage("http://8.8.8.8/"), "public ip");
        ok(!p.trustedPage("http://192.168.1.1.evil.com/"), "ip-looking name, never resolved");
        ok(!p.trustedPage("http://10.0.0.1@evil.com/"), "userinfo trick");
        ok(!p.trustedPage("http://evil.com\\@10.0.0.1/"), "backslash trick");
        ok(!p.trustedPage("http://[::1]/"), "ipv6 literal");
        ok(!p.trustedPage("https://192.168.0.209/"), "https is not the robot's bridge");
        ok(!p.trustedPage("intent://192.168.0.209/#Intent;scheme=http;end"), "intent:");
        ok(!p.trustedPage("javascript:alert(1)"), "javascript:");
        ok(!p.trustedPage("file:///data/data/x/y.html"), "file:");
        ok(!p.trustedPage(null) && !p.trustedPage(""), "null / empty");
        // names: not trusted until they resolved private (no DNS on the UI thread)
        ok(!p.trustedPage("http://router/"), "single label, not resolved yet");
        ok(!p.trustedPage("http://evil.lan/"), ".lan, not resolved yet");
        eq(HostPolicy.httpHost("http://OrangePi.LOCAL:8080/x"), "orangepi.local", "host lower-cased");
        eq(HostPolicy.hostPort("http://192.168.0.209/"), "192.168.0.209:80", "default port");
        eq(HostPolicy.hostPort("http://192.168.0.209:8080/"), "192.168.0.209:8080", "port");
        eq(HostPolicy.hostPort("https://192.168.0.209:8080/"), null, "https has no hostPort");
        eq(HostPolicy.schemeOf("INTENT://x"), "intent", "scheme");
        eq(HostPolicy.schemeOf("not a url"), "", "junk scheme");
    }

    static void names() throws Exception {
        FakeDns dns = new FakeDns();
        dns.set("router", "192.168.0.1");
        dns.set("evil.lan", "93.184.216.34");
        dns.set("mixed.lan", "192.168.0.7", "93.184.216.34");
        dns.set("robot.local", "192.168.0.209");
        HostPolicy p = new HostPolicy(dns, 10_000);
        p.setNow(1000);
        ok(!p.allow("evil.lan") && !p.trustedPage("http://evil.lan/"), ".lan name resolving public is refused");
        ok(!p.allow("mixed.lan"), "one public address is enough to refuse");
        ok(!p.allow("nothere"), "unresolvable name");
        ok(!p.trustedPage("http://router/"), "router: not trusted before allow()");
        ok(p.allow("router") && p.trustedPage("http://router/"), "router resolving private: allowed + pinned");
        int n = dns.lookups;
        p.allow("router");
        eq(dns.lookups, n, "answer cached inside the ttl");
        // rebinding: the name now also answers a different address -> refused for good (until clear())
        dns.set("robot.local", "192.168.0.209");
        ok(p.allow("robot.local"), "robot.local pinned");
        p.setNow(20_000);
        dns.set("robot.local", "192.168.0.209", "10.9.9.9");
        ok(!p.allow("robot.local"), "new address after pinning = rebinding");
        ok(!p.trustedPage("http://robot.local:8080/"), "rebound name loses trust");
        p.setNow(500_000);
        ok(!p.allow("robot.local"), "stays refused");
        p.clear();
        ok(p.allow("robot.local"), "network change forgets the pin");
        // a name that later turns public is refused at the next lookup
        dns.set("router", "8.8.8.8");
        p.setNow(900_000);
        ok(!p.allow("router"), "router now public");
        // literals never hit DNS
        int m = dns.lookups;
        ok(p.allow("192.168.5.5") && !p.allow("1.2.3.4") && p.allow("169.254.3.3"), "literals");
        eq(dns.lookups, m, "no lookup for literals");
        ok(HostPolicy.isPrivateAddr(InetAddress.getByName("fe80::1")) && HostPolicy.isPrivateAddr(InetAddress.getByName("fd00::1"))
            && HostPolicy.isPrivateAddr(InetAddress.getByName("::1")), "private ipv6");
        ok(!HostPolicy.isPrivateAddr(InetAddress.getByName("2001:4860:4860::8888")), "public ipv6");
    }

    // ------------------------------------------------------------------ #33: network choice, crash loop

    static void nets() {
        NetPick.Cand wifiNoNet = new NetPick.Cand("hotspot", "192.168.4.2", 24, false);
        NetPick.Cand wifiHome = new NetPick.Cand("home", "192.168.0.42", 24, true);
        NetPick.Cand eth = new NetPick.Cand("eth", "10.0.0.5", 8, false);
        eq(NetPick.pick(null, null), null, "none");
        eq(NetPick.pick(new ArrayList<NetPick.Cand>(), "1.2.3.4"), null, "empty");
        eq(NetPick.pick(Arrays.asList(wifiHome, wifiNoNet), "192.168.4.1").handle, "hotspot", "last robot's subnet wins over the default");
        eq(NetPick.pick(Arrays.asList(wifiHome, wifiNoNet), "192.168.0.209").handle, "home", "last robot on the default");
        eq(NetPick.pick(Arrays.asList(wifiNoNet, wifiHome), null).handle, "home", "no last: the default network");
        eq(NetPick.pick(Arrays.asList(wifiNoNet, eth), "172.16.0.1").handle, "hotspot", "nothing matches, none default: the first");
        ok(NetPick.sameSubnet("192.168.0.42", 24, "192.168.0.209"), "same /24");
        ok(!NetPick.sameSubnet("192.168.0.42", 24, "192.168.1.209"), "other /24");
        ok(NetPick.sameSubnet("10.0.0.5", 8, "10.200.1.1") && !NetPick.sameSubnet("10.0.0.5", 8, "11.0.0.1"), "/8");
        ok(NetPick.sameSubnet("192.168.0.130", 25, "192.168.0.200") && !NetPick.sameSubnet("192.168.0.130", 25, "192.168.0.20"), "/25");
        ok(!NetPick.sameSubnet("192.168.0.1", 24, "orangepi.local") && !NetPick.sameSubnet("x", 24, "192.168.0.1"), "not ips");
    }

    static void limit() {
        RateLimit r = new RateLimit(3, 60_000);
        ok(r.allow(0) && r.allow(1000) && r.allow(2000), "first three rebuilds");
        ok(!r.allow(3000) && !r.allow(30_000), "fourth inside a minute is refused (and not counted)");
        ok(r.allow(60_500), "the oldest left the window");
        ok(r.allow(61_500) && r.allow(62_500), "two more as the others age out");
        ok(!r.allow(63_000), "full again");
    }

    // ------------------------------------------------------------------ #31: gesture rects, rotation

    static void rects() {
        List<int[]> r = GestureRects.parse("[[100.5,200,300,400.2]]", 2f, 2400, 1080);
        eq(r.size(), 1, "one rect");
        eq(Arrays.toString(r.get(0)), "[201, 400, 600, 801]", "scaled by density, rounded outwards");
        r = GestureRects.parse("[[-50,-10,5000,9000]]", 3f, 2400, 1080);
        eq(Arrays.toString(r.get(0)), "[0, 0, 2400, 1080]", "clamped to the view");
        eq(GestureRects.parse("[]", 2f, 100, 100).size(), 0, "empty");
        eq(GestureRects.parse("[[10,10,10,50]]", 1f, 100, 100).size(), 0, "zero width dropped");
        eq(GestureRects.parse("[[500,10,600,50]]", 1f, 100, 100).size(), 0, "off view dropped");
        eq(GestureRects.parse("junk", 1f, 100, 100).size(), 0, "junk");
        eq(GestureRects.parse(null, 1f, 100, 100).size(), 0, "null");
        eq(GestureRects.parse("[[1,1,9,9]]", 0f, 100, 100).size(), 0, "bad density");
        eq(GestureRects.parse("[[1,1,9,9],[1,1,9,9],[1,1,9,9],[1,1,9,9],[1,1,9,9],[1,1,9,9]]", 1f, 100, 100).size(), GestureRects.MAX, "capped");
    }

    /** The same pose seen from the two landscape orientations is mirrored: a stale rotation inverts the drive. */
    static void tiltFrames() {
        float[] m = TiltMath.matrixFor(14, -9);
        double[] a = TiltMath.pitchRoll(m, 1), b = TiltMath.pitchRoll(m, 3);
        ok(Math.abs(a[0] + b[0]) < 0.05 && Math.abs(a[1] + b[1]) < 0.05, "rot 1 vs 3 are opposite: " + Arrays.toString(a) + Arrays.toString(b));
        ok(Math.abs(a[0]) > 5 || Math.abs(a[1]) > 5, "pose is not flat");
        double[] c = TiltMath.pitchRoll(m, 0), d = TiltMath.pitchRoll(m, 2);
        ok(Math.abs(c[0] + d[0]) < 0.05 && Math.abs(c[1] + d[1]) < 0.05, "rot 0 vs 2 are opposite");
        for (int rot = 0; rot < 4; rot++) {   // a 4-value domain: the app masks with & 3
            double[] x = TiltMath.pitchRoll(m, rot), y = TiltMath.pitchRoll(m, rot + 4);
            ok(Math.abs(x[0] - y[0]) < 1e-9 && Math.abs(x[1] - y[1]) < 1e-9, "rotation " + rot + " wraps");
        }
    }
}
