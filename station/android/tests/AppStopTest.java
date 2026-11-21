package com.sbr3.station;

import com.sun.net.httpserver.HttpServer;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/** Desktop-JVM tests for AppStop (the native stop-on-background sequence, issue #32). Run: tests\run.ps1 */
public class AppStopTest {
    static int fails, checks;

    static void ok(boolean c, String what) {
        checks++;
        if (!c) { fails++; System.out.println("FAIL " + what); }
    }

    static void eq(Object a, Object b, String what) { ok(a == null ? b == null : a.equals(b), what + ": " + a + " != " + b); }

    static String state(int mode, int status) {
        return "{\"status\": {\"t\": \"status\", \"link\": \"connected\", \"estop\": false}, \"tlm\": {\"mode\": " + mode
            + ", \"status\": " + status + ", \"pitch_deg\": 0.5}, \"nav\": null, \"ranges\": [], \"r\": 5}";
    }

    static void run() throws Exception {
        // ---- parse
        eq(Arrays.toString(AppStop.parseState(state(4, 3))), "[4, 3]", "parse goto");
        eq(Arrays.toString(AppStop.parseState("{\"status\": {\"status\": 9}, \"tlm\": {\"mode\":5,\"status\":1}}")), "[5, 1]", "compact, not the outer status");
        ok(AppStop.parseState("{\"status\": {}, \"tlm\": null}") == null, "no telemetry");
        ok(AppStop.parseState("{\"tlm\": {\"pitch_deg\": 1}}") == null, "no mode");
        ok(AppStop.parseState(null) == null && AppStop.parseState("") == null && AppStop.parseState("<html>") == null, "garbage");

        // ---- decision: only an armed, upright GOTO / EXPLORE goes to BALANCE
        final int A = AppStop.TLM_ARMED, U = AppStop.TLM_UPRIGHT;
        ok(AppStop.needsSafeMode(new int[]{AppStop.MODE_GOTO, A | U}), "goto");
        ok(AppStop.needsSafeMode(new int[]{AppStop.MODE_EXPLORE, A | U | 0x10}), "explore, other bits set");
        ok(!AppStop.needsSafeMode(new int[]{AppStop.MODE_GOTO, A}), "goto, not upright: never lift it");
        ok(!AppStop.needsSafeMode(new int[]{AppStop.MODE_EXPLORE, U}), "explore, not armed: never arm");
        ok(!AppStop.needsSafeMode(new int[]{AppStop.MODE_IDLE, 0}), "idle");
        ok(!AppStop.needsSafeMode(new int[]{AppStop.MODE_STATION, A | U}), "station: cmd_vel 0 is enough");
        ok(!AppStop.needsSafeMode(new int[]{AppStop.MODE_BALANCE, A | U}), "balance");
        ok(!AppStop.needsSafeMode(new int[]{AppStop.MODE_HOLD, A | U}), "hold");
        ok(!AppStop.needsSafeMode(new int[]{AppStop.MODE_MANUAL, A | U}), "manual (remote's)");
        ok(!AppStop.needsSafeMode(null), "unknown");

        // ---- command list: never an estop, never IDLE
        eq(AppStop.commands(new int[]{AppStop.MODE_EXPLORE, A | U}),
            Arrays.asList(AppStop.CMD_VEL_ZERO, AppStop.PLAN_STOP, AppStop.SET_BALANCE), "explore commands");
        eq(AppStop.commands(null), Arrays.asList(AppStop.CMD_VEL_ZERO, AppStop.PLAN_STOP), "unknown -> stops only");
        for (int m = 0; m < 7; m++) {
            for (String c : AppStop.commands(new int[]{m, A | U})) {
                ok(!c.contains("estop") && !c.contains("\"mode\":0"), "no estop / IDLE for mode " + m + ": " + c);
            }
        }

        // ---- the sequence against a fake bridge
        for (int[] st : new int[][]{{AppStop.MODE_GOTO, A | U}, {AppStop.MODE_EXPLORE, A | U}, {AppStop.MODE_STATION, A | U}, {AppStop.MODE_IDLE, 0}}) {
            final List<String> got = Collections.synchronizedList(new ArrayList<String>());
            HttpServer s = HttpServer.create(new InetSocketAddress("127.0.0.1", 0), 0);
            final String body = state(st[0], st[1]);
            s.createContext("/api/cmd", ex -> {
                got.add(ex.getRequestMethod() + " " + new String(readAll(ex.getRequestBody()), "UTF-8"));
                byte[] r = "{\"t\":\"ok\"}".getBytes("UTF-8");
                ex.sendResponseHeaders(200, r.length);
                ex.getResponseBody().write(r);
                ex.close();
            });
            s.createContext("/api/state", ex -> {
                got.add("GET " + ex.getRequestURI().getPath());
                byte[] r = body.getBytes("UTF-8");
                ex.sendResponseHeaders(200, r.length);
                ex.getResponseBody().write(r);
                ex.close();
            });
            s.start();
            try {
                List<String> sent = AppStop.stop("127.0.0.1:" + s.getAddress().getPort());
                boolean safe = AppStop.needsSafeMode(st);
                eq(sent, safe ? Arrays.asList(AppStop.CMD_VEL_ZERO, AppStop.PLAN_STOP, AppStop.SET_BALANCE)
                              : Arrays.asList(AppStop.CMD_VEL_ZERO, AppStop.PLAN_STOP), "accepted, mode " + st[0]);
                List<String> want = new ArrayList<>(Arrays.asList("POST " + AppStop.CMD_VEL_ZERO, "POST " + AppStop.PLAN_STOP, "GET /api/state"));
                if (safe) want.add("POST " + AppStop.SET_BALANCE);
                want.add("POST " + AppStop.CMD_VEL_ZERO);
                eq(got, want, "wire order, mode " + st[0]);
            } finally {
                s.stop(0);
            }
        }

        // ---- unreachable bridge: gives up after the first request, quickly
        long t0 = System.nanoTime();
        List<String> none = AppStop.stop("127.0.0.1:1");
        eq(none, new ArrayList<String>(), "nothing accepted");
        ok((System.nanoTime() - t0) / 1000000 < 3000, "gives up fast");
    }

    static byte[] readAll(InputStream in) throws java.io.IOException {
        ByteArrayOutputStream o = new ByteArrayOutputStream();
        byte[] b = new byte[512];
        int n;
        while ((n = in.read(b)) > 0) o.write(b, 0, n);
        return o.toByteArray();
    }

    public static void main(String[] args) throws Exception {
        run();
        System.out.println("AppStopTest: " + checks + " checks, " + fails + " failed");
        System.exit(fails == 0 ? 0 : 1);
    }
}
