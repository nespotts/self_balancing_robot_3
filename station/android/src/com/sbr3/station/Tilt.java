package com.sbr3.station;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.SystemClock;
import java.util.Locale;

/** Rotation-vector sensor -> tilt {pitch, roll, t} events (degrees, screen frame; see TiltMath) at up to `hz`. */
final class Tilt implements SensorEventListener {
    private final SensorManager sm;
    private final Sensor sensor;
    private final float[] r = new float[9];
    private volatile int rotation;
    private boolean wanted, running;
    private long periodMs = 33, lastEmit;

    Tilt(Context c) {
        sm = (SensorManager) c.getSystemService(Context.SENSOR_SERVICE);
        Sensor s = sm == null ? null : sm.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);   // no magnetometer: no compass jumps
        if (s == null && sm != null) s = sm.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        sensor = s;
    }

    boolean available() { return sensor != null; }

    void setRotation(int rot) { rotation = rot; }

    /** From the page (any thread). */
    synchronized void start(int hz) {
        periodMs = 1000L / Math.max(5, Math.min(60, hz <= 0 ? 30 : hz));
        wanted = true;
        resume();
    }

    synchronized void stop() {
        wanted = false;
        pause();
    }

    /** Activity resumed: start again if the page wants tilt. */
    synchronized void resume() {
        if (!wanted || running || sensor == null) return;
        running = sm.registerListener(this, sensor, (int) (periodMs * 1000));
    }

    /** Activity paused: the page sees the tilt go stale and stops driving. */
    synchronized void pause() {
        if (!running) return;
        sm.unregisterListener(this);
        running = false;
    }

    @Override public void onSensorChanged(SensorEvent e) {
        long now = SystemClock.uptimeMillis();
        if (now - lastEmit < periodMs - 4) return;
        lastEmit = now;
        SensorManager.getRotationMatrixFromVector(r, e.values);
        double[] pr = TiltMath.pitchRoll(r, rotation);
        Events.emit("tilt", String.format(Locale.ROOT, "{\"pitch\":%.2f,\"roll\":%.2f,\"t\":%d}", pr[0], pr[1], now));
    }

    @Override public void onAccuracyChanged(Sensor s, int accuracy) { }
}
