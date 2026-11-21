package com.sbr3.station;

/** Pure tilt math (unit-tested on the desktop JVM). */
final class TiltMath {
    private TiltMath() {}

    /**
     * Rotation matrix (device -> world, row-major 3x3 from SensorManager.getRotationMatrixFromVector) + display
     * rotation (0..3 = Surface.ROTATION_0..270) -> {pitch, roll} in degrees, in the frame of the screen as drawn:
     *   pitch + = top edge of the screen tilted away / down (drive forward)
     *   roll  + = right edge of the screen tilted down (turn right)
     * Flat, face up: 0, 0. The page zeroes both when the dead-man is pressed, so any holding angle works.
     */
    static double[] pitchRoll(float[] r, int rotation) {
        // world "up" in device coordinates = third row of R
        double ux = r[6], uy = r[7], uz = r[8];
        double sx, sy;
        switch (rotation & 3) {
            case 1: sx = -uy; sy = ux; break;    // ROTATION_90: screen right = device -Y, screen up = device +X
            case 2: sx = -ux; sy = -uy; break;
            case 3: sx = uy; sy = -ux; break;
            default: sx = ux; sy = uy;
        }
        double pitch = Math.toDegrees(Math.atan2(-sy, uz));
        double roll = Math.toDegrees(Math.atan2(-sx, Math.sqrt(sy * sy + uz * uz)));
        return new double[]{pitch, roll};
    }

    /** Rotation matrix for a phone rotated `pitchDeg` about the screen-x axis then `rollDeg` about screen-y (tests). */
    static float[] matrixFor(double pitchDeg, double rollDeg) {
        // up vector in device coords for: top edge tilted down by pitch, right edge down by roll (ROTATION_0)
        double p = Math.toRadians(pitchDeg), q = Math.toRadians(rollDeg);
        double ux = -Math.sin(q), uy = -Math.sin(p) * Math.cos(q), uz = Math.cos(p) * Math.cos(q);
        return new float[]{1, 0, 0, 0, 1, 0, (float) ux, (float) uy, (float) uz};
    }
}
