package com.sbr3.station;

/** Pure: at most `max` events per `windowMs` (WebView renderer rebuilds: a page that crashes on load must not loop). */
final class RateLimit {
    private final long[] times;
    private final long windowMs;
    private int n;

    RateLimit(int max, long windowMs) { times = new long[max]; this.windowMs = windowMs; }

    /** Record an event at `nowMs`; false = over the limit (the event is not counted). */
    synchronized boolean allow(long nowMs) {
        int k = 0;
        for (int i = 0; i < n; i++) if (nowMs - times[i] < windowMs) times[k++] = times[i];
        n = k;
        if (n >= times.length) return false;
        times[n++] = nowMs;
        return true;
    }
}
