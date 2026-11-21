package com.sbr3.station;

import java.util.List;

/** Desktop-JVM tests for the pure helpers (Lan, TiltMath). Run: tests\run.ps1 */
public class LanTest {
    static int fails, checks;

    static void ok(boolean c, String what) {
        checks++;
        if (!c) { fails++; System.out.println("FAIL " + what); }
    }

    static void eq(Object a, Object b, String what) { ok(a == null ? b == null : a.equals(b), what + ": " + a + " != " + b); }

    static void near(double a, double b, String what) { ok(Math.abs(a - b) < 0.05, what + ": " + a + " != " + b); }

    public static void main(String[] args) {
        // normalize
        eq(Lan.normalize("192.168.0.209"), "192.168.0.209:8080", "plain ip");
        eq(Lan.normalize(" http://192.168.0.209:8080/#drive "), "192.168.0.209:8080", "url");
        eq(Lan.normalize("OrangePi.local:9000"), "orangepi.local:9000", "host:port");
        eq(Lan.normalize("192.168.0.209:0"), null, "port 0");
        eq(Lan.normalize("192.168.0.209:99999"), null, "port too big");
        eq(Lan.normalize("a b"), null, "space");
        eq(Lan.normalize(""), null, "empty");
        eq(Lan.normalize(":8080"), null, "no host");
        eq(Lan.normalize("javascript:alert(1)"), null, "js url");

        // private hosts
        ok(Lan.isPrivateHost("192.168.0.209"), "192.168");
        ok(Lan.isPrivateHost("10.1.2.3"), "10/8");
        ok(Lan.isPrivateHost("172.16.0.1") && Lan.isPrivateHost("172.31.255.254"), "172.16/12");
        ok(!Lan.isPrivateHost("172.32.0.1") && !Lan.isPrivateHost("172.15.0.1"), "outside 172.16/12");
        ok(Lan.isPrivateHost("127.0.0.1") && Lan.isPrivateHost("169.254.1.1"), "loopback, link-local");
        ok(!Lan.isPrivateHost("8.8.8.8") && !Lan.isPrivateHost("192.169.0.1"), "public ip");
        ok(Lan.isPrivateHost("localhost") && Lan.isPrivateHost("orangepi.local") && Lan.isPrivateHost("orangepi"), "local names");
        ok(!Lan.isPrivateHost("example.com") && !Lan.isPrivateHost("evil.local.example.com"), "public names");
        ok(!Lan.isPrivateHost("256.1.1.1") && !Lan.isPrivateHost(""), "bad ip");

        // bridge status
        ok(Lan.looksLikeBridge("{\"t\": \"status\", \"link\": \"connected\", \"port\": \"/dev/ttyACM0\"}"), "status json");
        ok(Lan.looksLikeBridge("{\"t\":\"status\",\"link\":\"waiting\"}"), "compact status json");
        ok(!Lan.looksLikeBridge("<html>status link</html>"), "html");
        ok(!Lan.looksLikeBridge("{\"t\": \"hello\"}"), "other json");
        ok(!Lan.looksLikeBridge(null), "null");

        // scan list
        List<String> s = Lan.scanList("192.168.0.42", 24, "192.168.0.209");
        eq(s.size(), 253, "/24 minus phone, network, broadcast");
        eq(s.get(0), "192.168.0.209", "last robot first");
        ok(!s.contains("192.168.0.42") && !s.contains("192.168.0.0") && !s.contains("192.168.0.255"), "excluded");
        eq(s.indexOf("192.168.0.209"), s.lastIndexOf("192.168.0.209"), "no duplicate");
        eq(Lan.scanList("10.0.5.7", 16, null).size(), 253, "/16 capped at the /24");
        eq(Lan.scanList("10.0.5.7", 16, null).get(0), "10.0.5.1", "starts at .1");
        List<String> s28 = Lan.scanList("192.168.1.20", 28, "192.168.1.200");
        eq(s28.size(), 13, "/28: 14 hosts minus the phone");
        ok(!s28.contains("192.168.1.200"), "last robot outside the subnet is not added");
        eq(Lan.scanList("not-ip", 24, null).size(), 0, "no ip");
        eq(Lan.subnetLabel("192.168.0.42"), "192.168.0.x", "label");

        // tilt
        double[] pr = TiltMath.pitchRoll(TiltMath.matrixFor(0, 0), 0);
        near(pr[0], 0, "flat pitch");
        near(pr[1], 0, "flat roll");
        pr = TiltMath.pitchRoll(TiltMath.matrixFor(20, 0), 0);
        near(pr[0], 20, "top away pitch");
        near(pr[1], 0, "top away roll");
        pr = TiltMath.pitchRoll(TiltMath.matrixFor(0, 15), 0);
        near(pr[0], 0, "right down pitch");
        near(pr[1], 15, "right down roll");
        pr = TiltMath.pitchRoll(TiltMath.matrixFor(-40, 10), 0);
        near(pr[0], -40, "held up pitch");
        near(pr[1], 10, "held up roll");
        // landscape (ROTATION_90: device turned CCW, screen top = device left edge... screen up = device +X):
        // tilting the screen's top away = device +X edge down = device "right edge down" in portrait terms
        pr = TiltMath.pitchRoll(TiltMath.matrixFor(0, 12), 1);
        near(pr[0], 12, "rot90: device right edge down = screen top away");
        near(pr[1], 0, "rot90 roll");
        pr = TiltMath.pitchRoll(TiltMath.matrixFor(12, 0), 1);
        near(pr[1], -12, "rot90: device top edge down = screen left edge down");
        pr = TiltMath.pitchRoll(TiltMath.matrixFor(0, 12), 3);
        near(pr[0], -12, "rot270: device right edge down = screen top towards you");
        pr = TiltMath.pitchRoll(TiltMath.matrixFor(10, 5), 2);
        near(pr[0], -10, "rot180 pitch");
        near(pr[1], -5, "rot180 roll");

        System.out.println((fails == 0 ? "OK " : "FAILED ") + (checks - fails) + "/" + checks + " checks");
        System.exit(fails == 0 ? 0 : 1);
    }
}