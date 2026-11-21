package com.sbr3.station;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.content.pm.ApplicationInfo;
import android.content.pm.PackageInfo;
import android.content.res.Configuration;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.graphics.drawable.GradientDrawable;
import android.util.TypedValue;
import android.view.Display;
import android.view.Gravity;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowInsets;
import android.view.WindowInsetsController;
import android.view.WindowManager;
import android.webkit.ConsoleMessage;
import android.webkit.WebBackForwardList;
import android.webkit.WebChromeClient;
import android.webkit.WebResourceError;
import android.webkit.WebResourceRequest;
import android.webkit.WebResourceResponse;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.webkit.RenderProcessGoneDetail;
import android.widget.FrameLayout;
import android.widget.TextView;
import java.io.ByteArrayInputStream;
import java.net.InetAddress;
import java.net.URLEncoder;
import java.util.HashMap;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Full-screen WebView: the bundled find screen (assets/find.html), then the robot's own web app from the bridge
 * (http://<pi>:8080/). Adds window.SBR3Native (Bridge) for tilt, gamepad, haptics, keep-awake and orientation.
 */
public class MainActivity extends Activity {
    static final String TAG = "BalanceBot";
    static final String FINDER = "file:///android_asset/find.html";
    private static final int BG = 0xFF121212;   // the web app's --bg

    WebView web;
    private FrameLayout root;
    Finder finder;
    Tilt tilt;
    Gamepads pads;
    volatile boolean trusted;           // the shown page may use SBR3Native
    private volatile String robotAddr;  // host:port of the robot page being shown, or null
    private volatile String robotKind;         // Lan.ROBOT / Lan.SIM / null (unknown) for robotAddr
    private boolean labelShown;         // the "Robot · addr" label was flashed for this robot page
    private TextView label;
    private final ConcurrentHashMap<String, Boolean> hostOk = new ConcurrentHashMap<>();

    private SharedPreferences prefs() { return getSharedPreferences("sbr3", MODE_PRIVATE); }

    @Override protected void onCreate(Bundle b) {
        super.onCreate(b);
        finder = new Finder(this);
        WebView.setWebContentsDebuggingEnabled(isDebug());
        tilt = new Tilt(this);
        pads = new Gamepads(this);
        setOrientation(orientation());
        getWindow().getDecorView().setBackgroundColor(BG);
        if (Build.VERSION.SDK_INT >= 28) {
            WindowManager.LayoutParams a = getWindow().getAttributes();
            a.layoutInDisplayCutoutMode = WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_DEFAULT;  // keep clear of the camera
            getWindow().setAttributes(a);
        }
        root = new FrameLayout(this);
        root.setBackgroundColor(BG);
        setContentView(root);
        Events.attach(this);
        createWebView();
        loadFinder("");
        immersive();
    }

    // ------------------------------------------------------------------ app info / prefs

    boolean isDebug() { return (getApplicationInfo().flags & ApplicationInfo.FLAG_DEBUGGABLE) != 0; }

    String versionName() {
        try { return getPackageManager().getPackageInfo(getPackageName(), 0).versionName; } catch (Exception e) { return "?"; }
    }

    @SuppressWarnings("deprecation")
    long versionCode() {
        try {
            PackageInfo p = getPackageManager().getPackageInfo(getPackageName(), 0);
            return Build.VERSION.SDK_INT >= 28 ? p.getLongVersionCode() : p.versionCode;
        } catch (Exception e) {
            return 0;
        }
    }

    String lastAddress() { return prefs().getString("addr", null); }

    String orientation() { return prefs().getString("orientation", "auto"); }

    void setOrientation(String mode) {
        String m = "landscape".equals(mode) || "portrait".equals(mode) ? mode : "auto";
        prefs().edit().putString("orientation", m).apply();
        setRequestedOrientation("landscape".equals(m) ? ActivityInfo.SCREEN_ORIENTATION_SENSOR_LANDSCAPE
            : "portrait".equals(m) ? ActivityInfo.SCREEN_ORIENTATION_PORTRAIT : ActivityInfo.SCREEN_ORIENTATION_USER);
    }

    void keepScreenOn(boolean on) {
        if (on) getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        else getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    }

    // ------------------------------------------------------------------ pages

    private void createWebView() {
        web = new WebView(this);
        web.setBackgroundColor(BG);
        web.setOverScrollMode(View.OVER_SCROLL_NEVER);
        WebSettings s = web.getSettings();
        s.setJavaScriptEnabled(true);
        s.setDomStorageEnabled(true);
        s.setGeolocationEnabled(false);
        s.setAllowFileAccess(false);            // file:///android_asset still works
        s.setAllowContentAccess(false);
        s.setSupportZoom(false);
        s.setBuiltInZoomControls(false);
        s.setTextZoom(100);
        s.setMediaPlaybackRequiresUserGesture(true);
        if (Build.VERSION.SDK_INT >= 33) s.setAlgorithmicDarkeningAllowed(false);
        s.setUserAgentString(s.getUserAgentString() + " BalanceBot/" + versionName());
        web.setWebViewClient(new Client());
        web.setWebChromeClient(new Chrome());
        web.addJavascriptInterface(new Bridge(this), "SBR3Native");
        root.addView(web, 0, new FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT));
    }

    /** `hash` e.g. "" / "#error=...&addr=..." / "?manual=1" (turned into a hash: asset URLs ignore queries). */
    void loadFinder(String hash) {
        finder.stop();
        tilt.stop();
        keepScreenOn(false);
        robotAddr = null;
        robotKind = null;
        hideRobotLabel();
        trusted = true;
        String h = hash == null ? "" : hash.startsWith("?") ? "#" + hash.substring(1) : hash;
        web.loadUrl(FINDER + h);
    }

    void openRobot(String addr) {
        finder.stop();
        String k = finder.kindOf(addr);
        if (k == null && addr.equals(lastAddress())) k = prefs().getString("kind", null);
        prefs().edit().putString("addr", addr).putString("kind", k).apply();
        robotAddr = addr;
        robotKind = k;
        labelShown = false;
        web.loadUrl("http://" + addr + "/");
    }

    String robotAddr() { return robotAddr; }

    String robotKind() { return robotKind; }

    /** "Robot · 192.168.0.209" / "Simulator · 192.168.0.149" for the robot page shown, or null. */
    String robotLabel() {
        String a = robotAddr;
        if (a == null) return null;
        String host = a.endsWith(":" + Lan.PORT) ? Lan.hostOf(a) : a;
        return robotKind == null ? host : Lan.kindLabel(robotKind) + " · " + host;
    }

    /** Small label at the top for a few seconds after the robot page opens: which robot this is. Never takes touches. */
    private void flashRobotLabel() {
        String text = robotLabel();
        if (text == null) return;
        if (label == null) {
            label = new TextView(this);
            float dp = getResources().getDisplayMetrics().density;
            label.setTextColor(0xFFF0EFEC);
            label.setTextSize(TypedValue.COMPLEX_UNIT_SP, 13);
            int ph = Math.round(12 * dp), pv = Math.round(5 * dp);
            label.setPadding(ph, pv, ph, pv);
            GradientDrawable bg = new GradientDrawable();
            bg.setColor(0xE6242422);
            bg.setStroke(Math.max(1, Math.round(dp)), 0xFF3A3A37);
            bg.setCornerRadius(14 * dp);
            label.setBackground(bg);
            label.setClickable(false);
            label.setFocusable(false);
            FrameLayout.LayoutParams lp = new FrameLayout.LayoutParams(FrameLayout.LayoutParams.WRAP_CONTENT,
                FrameLayout.LayoutParams.WRAP_CONTENT, Gravity.TOP | Gravity.CENTER_HORIZONTAL);
            lp.topMargin = Math.round(64 * dp);   // below the web app's 56-60 px top bar
            root.addView(label, lp);
        }
        label.setText(text);
        label.animate().cancel();
        label.setAlpha(1f);
        label.setVisibility(View.VISIBLE);
        label.animate().alpha(0f).setStartDelay(4000).setDuration(600)
            .withEndAction(() -> label.setVisibility(View.GONE)).start();
    }

    private void hideRobotLabel() {
        if (label == null) return;
        label.animate().cancel();
        label.setVisibility(View.GONE);
    }

    static boolean isTrusted(String url) {
        if (url == null) return false;
        if (url.startsWith(FINDER)) return true;
        try {
            Uri u = Uri.parse(url);
            return "http".equals(u.getScheme()) && Lan.isPrivateHost(u.getHost());
        } catch (Exception e) {
            return false;
        }
    }

    /** Plain http only to private-LAN hosts (Android's network security config can't express IP ranges). Background thread. */
    private boolean hostAllowed(String host) {
        if (host == null) return false;
        int[] ip = Lan.ipv4(host);
        if (ip != null) return Lan.isPrivateIpv4(ip);
        Boolean c = hostOk.get(host);
        if (c != null) return c;
        boolean ok = false;
        try {
            InetAddress[] all = InetAddress.getAllByName(host);
            ok = all.length > 0;
            for (InetAddress a : all) ok &= a.isSiteLocalAddress() || a.isLoopbackAddress() || a.isLinkLocalAddress();
        } catch (Exception ignored) {
        }
        hostOk.put(host, ok);
        return ok;
    }

    void evalJs(String js) {
        if (web != null && trusted) web.evaluateJavascript(js, null);
    }

    // ------------------------------------------------------------------ window

    private void immersive() {
        if (Build.VERSION.SDK_INT >= 30) {
            WindowInsetsController c = getWindow().getInsetsController();
            if (c != null) {
                c.hide(WindowInsets.Type.systemBars());
                c.setSystemBarsBehavior(WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE);
            }
        } else {
            getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                | View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION | View.SYSTEM_UI_FLAG_LAYOUT_STABLE);
        }
    }

    @Override public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) immersive();
    }

    @SuppressWarnings("deprecation")
    private int displayRotation() {
        Display d = Build.VERSION.SDK_INT >= 30 ? getDisplay() : getWindowManager().getDefaultDisplay();
        return d == null ? 0 : d.getRotation();
    }

    @Override public void onConfigurationChanged(Configuration c) {
        super.onConfigurationChanged(c);
        tilt.setRotation(displayRotation());
    }

    // ------------------------------------------------------------------ lifecycle

    @Override protected void onResume() {
        super.onResume();
        tilt.setRotation(displayRotation());
        tilt.resume();
        pads.register();
        if (web != null) web.onResume();
        Events.emit("resume", "{}");
        Events.emit("gamepads", pads.listJson());
        immersive();
    }

    @Override protected void onPause() {
        // the page releases the dead-man and stops cmd_vel on "pause"; tilt + gamepad also go quiet (stale -> stop).
        // That is page JS, which a frozen / killed WebView never runs: stop from native code too (issue #32)
        stopRobot();
        Events.emit("pause", "{}");
        tilt.pause();
        pads.zero();
        pads.unregister();
        if (web != null) web.onPause();
        super.onPause();
    }

    /** Native stop for the robot page being shown: zero cmd_vel, stop Go to / Explore, keep balancing (AppStop). */
    void stopRobot() {
        String a = robotAddr;
        if (a != null) AppStop.run(a);
    }

    @Override protected void onDestroy() {
        stopRobot();
        finder.stop();
        tilt.stop();
        Events.detach(this);
        if (web != null) {
            root.removeView(web);
            web.destroy();
            web = null;
        }
        super.onDestroy();
    }

    @SuppressWarnings("deprecation")
    @Override public void onBackPressed() {
        if (web == null || robotAddr == null) { finish(); return; }
        WebBackForwardList h = web.copyBackForwardList();
        int i = h.getCurrentIndex();
        if (i > 0 && !h.getItemAtIndex(i - 1).getUrl().startsWith(FINDER)) { web.goBack(); return; }
        new AlertDialog.Builder(this, android.R.style.Theme_Material_Dialog_Alert)
            .setTitle("Leave the robot?")
            .setMessage("Driving, Go to and Explore stop; balancing stays on.")
            .setPositiveButton("Exit", (d, w) -> finish())
            .setNeutralButton("Find robot", (d, w) -> loadFinder("?manual=1"))
            .setNegativeButton("Stay", null)
            .show();
    }

    // ------------------------------------------------------------------ gamepad input

    @Override public boolean dispatchKeyEvent(KeyEvent e) {
        if (Gamepads.isPadKey(e)) return pads.onKey(e);
        return super.dispatchKeyEvent(e);
    }

    @Override public boolean dispatchGenericMotionEvent(MotionEvent e) {
        if (Gamepads.isPadMotion(e)) return pads.onMotion(e);
        return super.dispatchGenericMotionEvent(e);
    }

    // ------------------------------------------------------------------ WebView clients

    private final class Client extends WebViewClient {
        @Override public boolean shouldOverrideUrlLoading(WebView v, WebResourceRequest r) {
            String url = r.getUrl().toString();
            if (isTrusted(url)) return false;
            try {
                startActivity(new Intent(Intent.ACTION_VIEW, r.getUrl()).addFlags(Intent.FLAG_ACTIVITY_NEW_TASK));
            } catch (Exception e) {
                Log.w(TAG, "open " + url + ": " + e);
            }
            return true;
        }

        @Override public WebResourceResponse shouldInterceptRequest(WebView v, WebResourceRequest r) {
            Uri u = r.getUrl();
            if ("http".equals(u.getScheme()) && !hostAllowed(u.getHost())) {
                Log.w(TAG, "blocked cleartext request to " + u.getHost());
                return new WebResourceResponse("text/plain", "utf-8", 403, "Blocked", new HashMap<String, String>(),
                    new ByteArrayInputStream(new byte[0]));
            }
            return null;
        }

        @Override public void onPageStarted(WebView v, String url, android.graphics.Bitmap icon) {
            trusted = isTrusted(url);
        }

        @Override public void onPageFinished(WebView v, String url) {
            if (robotAddr != null && !labelShown && url != null && url.startsWith("http://" + robotAddr + "/")) {
                labelShown = true;
                flashRobotLabel();
            }
        }

        @Override public void onReceivedError(WebView v, WebResourceRequest r, WebResourceError err) {
            if (!r.isForMainFrame() || robotAddr == null) return;
            fail(robotAddr, String.valueOf(err.getDescription()));
        }

        @Override public void onReceivedHttpError(WebView v, WebResourceRequest r, WebResourceResponse resp) {
            if (!r.isForMainFrame() || robotAddr == null) return;
            fail(robotAddr, "HTTP " + resp.getStatusCode());
        }

        private void fail(String addr, String why) {
            try {
                loadFinder("#error=" + URLEncoder.encode(why, "UTF-8") + "&addr=" + URLEncoder.encode(addr, "UTF-8"));
            } catch (Exception e) {
                loadFinder("#error=1");
            }
        }

        @Override public boolean onRenderProcessGone(WebView v, RenderProcessGoneDetail d) {
            Log.e(TAG, "WebView renderer gone (crash=" + d.didCrash() + "); rebuilding it");
            if (v == web) {
                stopRobot();   // the page that was driving is gone; no JS stop will come
                String addr = robotAddr;
                root.removeView(web);
                web.destroy();
                createWebView();
                if (addr != null) openRobot(addr); else loadFinder("");
            }
            return true;
        }
    }

    private static final class Chrome extends WebChromeClient {
        @Override public boolean onConsoleMessage(ConsoleMessage m) {
            Log.i("BalanceBotWeb", m.message() + " (" + m.sourceId() + ":" + m.lineNumber() + ")");
            return true;
        }
    }
}
