package com.sbr3.station;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Stop the robot from the native side when the app is left (onPause / onDestroy / renderer gone), so it does not
 * depend on the page's JS (a paused or killed WebView never runs it). Best effort, from a short-lived thread:
 *   1. cmd_vel 0,0           (always)
 *   2. plan stop             (always; harmless when no Go to / Explore is running)
 *   3. set_mode BALANCE      only if the robot is in GOTO or EXPLORE and armed + upright
 * "Safe" = stop moving but keep balancing. Never E-STOP / IDLE (that drops a balancing robot) and never arms:
 * a robot that is not armed and upright is left alone. STATION / BALANCE / HOLD already stand still with cmd_vel 0.
 * The pure decision logic (parse, decide, commands) has no Android dependencies and is unit-tested on the JVM.
 */
final class AppStop {
    static final int MODE_IDLE = 0, MODE_BALANCE = 1, MODE_MANUAL = 2, MODE_STATION = 3, MODE_GOTO = 4, MODE_EXPLORE = 5, MODE_HOLD = 6;
    static final int TLM_ARMED = 0x01, TLM_UPRIGHT = 0x02;

    static final String CMD_VEL_ZERO = "{\"t\":\"cmd_vel\",\"speed\":0,\"turn\":0}";
    static final String PLAN_STOP = "{\"t\":\"plan\",\"action\":\"stop\"}";
    static final String SET_BALANCE = "{\"t\":\"set_mode\",\"mode\":" + MODE_BALANCE + "}";

    private static final int CONNECT_MS = 700, READ_MS = 900;
    private static final long MIN_GAP_MS = 1500;
    private static final AtomicLong lastRun = new AtomicLong(-MIN_GAP_MS);

    private AppStop() { }

    // ------------------------------------------------------------------ pure logic

    /** {mode, status} from a GET /api/state body (its "tlm" object), or null when there is no telemetry. */
    static int[] parseState(String body) {
        if (body == null) return null;
        Matcher t = Pattern.compile("\"tlm\"\\s*:\\s*\\{").matcher(body);
        if (!t.find()) return null;
        int end = body.indexOf('}', t.end());
        String tlm = body.substring(t.end(), end < 0 ? body.length() : end);
        Matcher m = Pattern.compile("\"mode\"\\s*:\\s*(\\d+)").matcher(tlm);
        Matcher s = Pattern.compile("\"status\"\\s*:\\s*(\\d+)").matcher(tlm);
        if (!m.find() || !s.find()) return null;
        try {
            return new int[]{Integer.parseInt(m.group(1)), Integer.parseInt(s.group(1))};
        } catch (NumberFormatException e) {
            return null;
        }
    }

    /** Leave GOTO / EXPLORE for BALANCE? Only while armed and upright, so this can never arm or lift a fallen robot. */
    static boolean needsSafeMode(int[] state) {
        if (state == null) return false;
        boolean auto = state[0] == MODE_GOTO || state[0] == MODE_EXPLORE;
        return auto && (state[1] & TLM_ARMED) != 0 && (state[1] & TLM_UPRIGHT) != 0;
    }

    /** Commands to send for a given robot state, in order. state == null (unknown) -> just the stops. */
    static List<String> commands(int[] state) {
        List<String> c = new ArrayList<>();
        c.add(CMD_VEL_ZERO);
        c.add(PLAN_STOP);
        if (needsSafeMode(state)) c.add(SET_BALANCE);
        return c;
    }

    // ------------------------------------------------------------------ network (background thread)

    /** Fire and forget: runs the stop sequence against host:port on its own thread. Skips a repeat within 1.5 s. */
    static void run(final String hostPort) {
        if (hostPort == null) return;
        long now = System.nanoTime() / 1000000L;
        long last = lastRun.get();
        if (now - last < MIN_GAP_MS || !lastRun.compareAndSet(last, now)) return;
        Thread t = new Thread(() -> stop(hostPort), "app-stop");
        t.setDaemon(false);
        t.start();
    }

    /** The whole sequence, blocking (short timeouts). Returns the commands that were accepted (for tests). */
    static List<String> stop(String hostPort) {
        List<String> sent = new ArrayList<>();
        // the zero first and on its own: if the robot's bridge is unreachable there is nothing more to do
        if (!post(hostPort, CMD_VEL_ZERO)) return sent;
        sent.add(CMD_VEL_ZERO);
        if (post(hostPort, PLAN_STOP)) sent.add(PLAN_STOP);
        int[] st = parseState(get(hostPort, "/api/state?max=0&r=2000000000"));
        if (needsSafeMode(st) && post(hostPort, SET_BALANCE)) sent.add(SET_BALANCE);
        post(hostPort, CMD_VEL_ZERO);   // and once more last, in case the page's own zero raced the mode change
        return sent;
    }

    static boolean post(String hostPort, String json) {
        HttpURLConnection c = null;
        try {
            c = open(hostPort, "/api/cmd");
            c.setRequestMethod("POST");
            c.setDoOutput(true);
            c.setRequestProperty("Content-Type", "application/json");
            byte[] b = json.getBytes("UTF-8");
            c.setFixedLengthStreamingMode(b.length);
            try (OutputStream o = c.getOutputStream()) { o.write(b); }
            int code = c.getResponseCode();
            drain(c);
            return code == 200;
        } catch (Exception e) {
            return false;
        } finally {
            if (c != null) c.disconnect();
        }
    }

    static String get(String hostPort, String path) {
        HttpURLConnection c = null;
        try {
            c = open(hostPort, path);
            if (c.getResponseCode() != 200) return null;
            try (InputStream in = c.getInputStream()) {
                java.io.ByteArrayOutputStream out = new java.io.ByteArrayOutputStream();
                byte[] buf = new byte[4096];
                int n;
                while ((n = in.read(buf)) > 0 && out.size() < 65536) out.write(buf, 0, n);
                return out.toString("UTF-8");
            }
        } catch (Exception e) {
            return null;
        } finally {
            if (c != null) c.disconnect();
        }
    }

    private static HttpURLConnection open(String hostPort, String path) throws Exception {
        HttpURLConnection c = (HttpURLConnection) new URL("http://" + hostPort + path).openConnection();
        c.setConnectTimeout(CONNECT_MS);
        c.setReadTimeout(READ_MS);
        c.setUseCaches(false);
        c.setInstanceFollowRedirects(false);
        return c;
    }

    private static void drain(HttpURLConnection c) {
        try (InputStream in = c.getInputStream()) {
            byte[] buf = new byte[1024];
            while (in.read(buf) > 0) { /* discard */ }
        } catch (Exception ignored) {
        }
    }
}
