package com.sbr3.station;

import android.content.Context;
import android.hardware.input.InputManager;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.MotionEvent;
import java.util.Locale;
import org.json.JSONArray;
import org.json.JSONObject;

/**
 * Bluetooth / USB gamepads -> page events (UI thread):
 *   gamepad {lx, ly, rx, ry, hx, hy, lt, rt, t}  axes -1..1 (y + = down, as Android reports), on change and every
 *                                                100 ms while any stick is off centre (the page treats >400 ms as gone)
 *   gamepadButton {name: "A"|"B"|"X"|"Y"|"L1"|"R1"|"START"|"SELECT"|..., down}
 *   gamepads {list: [{id, name}]}                on connect / disconnect
 */
final class Gamepads implements InputManager.InputDeviceListener {
    private final InputManager im;
    private final Handler ui = new Handler(Looper.getMainLooper());
    private final float[] ax = new float[8];   // lx ly rx ry hx hy lt rt
    private long lastEmit;
    private final Runnable repeat = new Runnable() {
        @Override public void run() {
            if (!centred()) { emitAxes(); ui.postDelayed(this, 100); }
        }
    };

    Gamepads(Context c) {
        im = (InputManager) c.getSystemService(Context.INPUT_SERVICE);
    }

    void register() { if (im != null) im.registerInputDeviceListener(this, ui); }

    void unregister() { if (im != null) im.unregisterInputDeviceListener(this); }

    static boolean isPad(InputDevice d) {
        if (d == null || d.isVirtual()) return false;
        int s = d.getSources();
        return (s & InputDevice.SOURCE_GAMEPAD) == InputDevice.SOURCE_GAMEPAD
            || (s & InputDevice.SOURCE_JOYSTICK) == InputDevice.SOURCE_JOYSTICK;
    }

    static boolean isPadKey(KeyEvent e) {
        int s = e.getSource();
        return ((s & InputDevice.SOURCE_GAMEPAD) == InputDevice.SOURCE_GAMEPAD
            || (s & InputDevice.SOURCE_JOYSTICK) == InputDevice.SOURCE_JOYSTICK) && KeyEvent.isGamepadButton(e.getKeyCode());
    }

    static boolean isPadMotion(MotionEvent e) {
        return (e.getSource() & InputDevice.SOURCE_JOYSTICK) == InputDevice.SOURCE_JOYSTICK
            && e.getAction() == MotionEvent.ACTION_MOVE;
    }

    String listJson() {
        JSONArray a = new JSONArray();
        if (im != null) {
            for (int id : im.getInputDeviceIds()) {
                InputDevice d = im.getInputDevice(id);
                if (!isPad(d)) continue;
                try { a.put(new JSONObject().put("id", id).put("name", d.getName())); } catch (Exception ignored) { }
            }
        }
        try { return new JSONObject().put("list", a).toString(); } catch (Exception e) { return "{\"list\":[]}"; }
    }

    private static float centred(MotionEvent e, InputDevice d, int axis) {
        InputDevice.MotionRange r = d == null ? null : d.getMotionRange(axis, e.getSource());
        if (r == null) return 0;
        float v = e.getAxisValue(axis);
        return Math.abs(v) > r.getFlat() ? Math.max(-1, Math.min(1, v)) : 0;
    }

    boolean onMotion(MotionEvent e) {
        InputDevice d = e.getDevice();
        float lt = Math.max(centred(e, d, MotionEvent.AXIS_LTRIGGER), centred(e, d, MotionEvent.AXIS_BRAKE));
        float rt = Math.max(centred(e, d, MotionEvent.AXIS_RTRIGGER), centred(e, d, MotionEvent.AXIS_GAS));
        float[] n = {centred(e, d, MotionEvent.AXIS_X), centred(e, d, MotionEvent.AXIS_Y),
            centred(e, d, MotionEvent.AXIS_Z), centred(e, d, MotionEvent.AXIS_RZ),
            centred(e, d, MotionEvent.AXIS_HAT_X), centred(e, d, MotionEvent.AXIS_HAT_Y), lt, rt};
        boolean changed = false;
        for (int i = 0; i < 8; i++) if (Math.abs(n[i] - ax[i]) > 0.01f) { changed = true; ax[i] = n[i]; }
        long now = SystemClock.uptimeMillis();
        if (changed && (now - lastEmit >= 25 || centred())) emitAxes();   // ~40 Hz max, centring always goes out
        ui.removeCallbacks(repeat);
        if (!centred()) ui.postDelayed(repeat, 100);
        return true;
    }

    private boolean centred() {
        for (int i = 0; i < 6; i++) if (ax[i] != 0) return false;
        return true;
    }

    private void emitAxes() {
        lastEmit = SystemClock.uptimeMillis();
        Events.emit("gamepad", String.format(Locale.ROOT,
            "{\"lx\":%.3f,\"ly\":%.3f,\"rx\":%.3f,\"ry\":%.3f,\"hx\":%.0f,\"hy\":%.0f,\"lt\":%.3f,\"rt\":%.3f,\"t\":%d}",
            ax[0], ax[1], ax[2], ax[3], ax[4], ax[5], ax[6], ax[7], lastEmit));
    }

    /** App paused / pad gone: everything back to centre. */
    void zero() {
        ui.removeCallbacks(repeat);
        java.util.Arrays.fill(ax, 0);
        emitAxes();
    }

    static String buttonName(int code) {
        switch (code) {
            case KeyEvent.KEYCODE_BUTTON_A: return "A";
            case KeyEvent.KEYCODE_BUTTON_B: return "B";
            case KeyEvent.KEYCODE_BUTTON_X: return "X";
            case KeyEvent.KEYCODE_BUTTON_Y: return "Y";
            case KeyEvent.KEYCODE_BUTTON_L1: return "L1";
            case KeyEvent.KEYCODE_BUTTON_R1: return "R1";
            case KeyEvent.KEYCODE_BUTTON_L2: return "L2";
            case KeyEvent.KEYCODE_BUTTON_R2: return "R2";
            case KeyEvent.KEYCODE_BUTTON_THUMBL: return "L3";
            case KeyEvent.KEYCODE_BUTTON_THUMBR: return "R3";
            case KeyEvent.KEYCODE_BUTTON_START: return "START";
            case KeyEvent.KEYCODE_BUTTON_SELECT: return "SELECT";
            case KeyEvent.KEYCODE_BUTTON_MODE: return "MODE";
            default: return "KEY" + code;
        }
    }

    boolean onKey(KeyEvent e) {
        if (e.getRepeatCount() > 0) return true;
        int a = e.getAction();
        if (a != KeyEvent.ACTION_DOWN && a != KeyEvent.ACTION_UP) return true;
        Events.emit("gamepadButton", "{\"name\":\"" + buttonName(e.getKeyCode()) + "\",\"down\":" + (a == KeyEvent.ACTION_DOWN) + "}");
        return true;
    }

    @Override public void onInputDeviceAdded(int id) {
        if (isPad(im.getInputDevice(id))) Events.emit("gamepads", listJson());
    }

    @Override public void onInputDeviceRemoved(int id) {
        zero();
        Events.emit("gamepads", listJson());
    }

    @Override public void onInputDeviceChanged(int id) { }
}
