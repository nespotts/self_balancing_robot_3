package com.sbr3.station;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Pure (desktop-JVM tested): the drive pad's rectangles as the page reports them, "[[l,t,r,b],...]" in CSS px of
 * the viewport -> view pixels, clamped to the view, for View.setSystemGestureExclusionRects (no Back swipe from the pad).
 */
final class GestureRects {
    private static final Pattern NUM = Pattern.compile("-?[0-9]+(?:\\.[0-9]+)?(?:[eE][-+]?[0-9]+)?");
    static final int MAX = 4;

    private GestureRects() {}

    /** Each result is {left, top, right, bottom}. Junk, empty and off-view rects are dropped. */
    static List<int[]> parse(String json, float density, int viewW, int viewH) {
        List<int[]> out = new ArrayList<>();
        if (json == null || density <= 0 || viewW <= 0 || viewH <= 0) return out;
        Matcher m = NUM.matcher(json);
        double[] v = new double[4];
        int i = 0;
        while (m.find() && out.size() < MAX) {
            double d;
            try { d = Double.parseDouble(m.group()); } catch (NumberFormatException e) { return new ArrayList<>(); }
            if (Double.isNaN(d) || Double.isInfinite(d)) return new ArrayList<>();
            v[i++] = d;
            if (i < 4) continue;
            i = 0;
            int l = clamp((int) Math.floor(v[0] * density), viewW), t = clamp((int) Math.floor(v[1] * density), viewH);
            int r = clamp((int) Math.ceil(v[2] * density), viewW), b = clamp((int) Math.ceil(v[3] * density), viewH);
            if (r > l && b > t) out.add(new int[]{l, t, r, b});
        }
        return out;
    }

    private static int clamp(int x, int max) { return Math.max(0, Math.min(max, x)); }
}
