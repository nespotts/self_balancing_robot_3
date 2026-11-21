package com.sbr3.station;

import org.json.JSONObject;

/** Java -> page: window.__sbr3.emit(name, jsonString), on the UI thread, only into a trusted page. */
final class Events {
    private static volatile MainActivity act;

    private Events() {}

    static void attach(MainActivity a) { act = a; }

    static void detach(MainActivity a) { if (act == a) act = null; }

    static void emit(String name, String json) {
        final MainActivity a = act;
        if (a == null) return;
        final String js = "window.__sbr3&&window.__sbr3.emit('" + name + "'," + JSONObject.quote(json) + ")";
        a.runOnUiThread(() -> a.evalJs(js));
    }

    static void emit(String name, JSONObject o) { emit(name, o.toString()); }
}
