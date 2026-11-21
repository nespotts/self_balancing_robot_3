package com.sbr3.station;

import android.os.Build;
import android.content.Context;
import android.webkit.JavascriptInterface;
import org.json.JSONObject;

/**
 * window.SBR3Native (page -> Java). Only answers while a trusted page is shown (the bundled find screen or a
 * private-LAN robot page; see MainActivity.trusted / HostPolicy). Events come back through window.__sbr3.emit(name, json).
 */
final class Bridge {
    private final MainActivity act;

    Bridge(MainActivity a) { act = a; }

    private boolean ok() { return act.trusted(); }

    @JavascriptInterface public String appInfo() {
        if (!ok()) return "{}";
        try {
            return new JSONObject().put("app", "Balance Bot").put("version", act.versionName()).put("code", act.versionCode())
                .put("debug", act.isDebug()).put("sdk", Build.VERSION.SDK_INT).put("model", Build.MODEL)
                .put("tilt", act.tilt.available()).put("orientation", act.orientation())
                .put("addr", act.lastAddress() == null ? JSONObject.NULL : act.lastAddress())
                .put("robot", act.robotAddr() == null ? JSONObject.NULL : new JSONObject().put("addr", act.robotAddr())
                    .put("kind", act.robotKind() == null ? JSONObject.NULL : act.robotKind()).put("label", act.robotLabel()))
                .toString();
        } catch (Exception e) {
            return "{}";
        }
    }

    // ---------------------------------------------------------------- finding the robot

    @JavascriptInterface public String lastAddress() { return ok() && act.lastAddress() != null ? act.lastAddress() : ""; }

    @JavascriptInterface public void findStart() { if (ok()) act.finder.start(act.lastAddress()); }

    @JavascriptInterface public void findStop() { if (ok()) act.finder.stop(); }

    @JavascriptInterface public void probe(String addr) { if (ok()) act.finder.probe(addr); }

    /** Remember the address and open the robot's web app. */
    @JavascriptInterface public void connect(String addr) {
        if (!ok()) return;
        final String a = Lan.normalize(addr);
        if (a == null || !Lan.isPrivateHost(Lan.hostOf(a))) return;
        act.runOnUiThread(() -> act.openRobot(a));
    }

    /** Back to the find screen (e.g. "Find another robot" in the Drive ⋯ menu). */
    @JavascriptInterface public void openFinder() {
        if (ok()) act.runOnUiThread(() -> act.loadFinder("?manual=1"));
    }

    // ---------------------------------------------------------------- sensors / inputs

    @JavascriptInterface public void tiltStart(int hz) { if (ok()) act.tilt.start(hz); }

    @JavascriptInterface public void tiltStop() { act.tilt.stop(); }   // always allowed: stopping is safe

    @JavascriptInterface public String gamepads() { return ok() ? act.pads.listJson() : "{\"list\":[]}"; }

    // ---------------------------------------------------------------- phone

    @JavascriptInterface public void keepAwake(boolean on) {
        if (ok()) act.runOnUiThread(() -> act.keepScreenOn(on));
    }

    /** "auto" (follows the phone, ignores the rotation lock), "landscape", "portrait". Remembered. */
    @JavascriptInterface public void setOrientation(String mode) {
        if (ok()) act.runOnUiThread(() -> act.setOrientation(mode));
    }

    /** The drive pad's rectangles [[l,t,r,b],..] in CSS px: the system Back swipe doesn't start from them (API 29+). */
    @JavascriptInterface public void gestureRects(String json) {
        if (ok() && json != null && json.length() < 512) act.runOnUiThread(() -> act.setGestureRects(json));
    }

    @JavascriptInterface public String getOrientation() { return act.orientation(); }
}
