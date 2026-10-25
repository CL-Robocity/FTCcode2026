package org.firstinspires.ftc.teamcode.Odometry;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Legge il JSON creato dall'HUD e lo trasforma in un RobotPath (cm, radianti).
 *
 * Formato:
 * {
 *   "units": "in" | "cm",                       (default "in")
 *   "start": { "x": 24, "y": 24, "heading": 0 }, (heading in gradi, 0 = verso +Y, antiorario +)
 *   "segments": [
 *     { "type": "line",  "to": [x, y], "heading": "tangent" | gradi | (omesso = mantiene) },
 *     { "type": "curve", "controls": [[x, y], ...], "to": [x, y], "heading": ... }
 *   ]
 * }
 * Le curve sono Bezier: 1 controllo = quadratica, 2 = cubica, ecc.
 */
public class PathLoader {

    public static final String FOLDER = "/sdcard/FIRST/paths/";
    private static final double SAMPLE_STEP_CM = 2.0;

    public static RobotPath load(String fileName) throws IOException {
        StringBuilder sb = new StringBuilder();
        try (BufferedReader r = new BufferedReader(new FileReader(new File(FOLDER + fileName)))) {
            String line;
            while ((line = r.readLine()) != null) sb.append(line).append('\n');
        }
        return parse(sb.toString());
    }

    public static RobotPath parse(String json) {
        JsonObject root = new JsonParser().parse(json).getAsJsonObject();
        double k = unitFactor(root);

        JsonObject st = root.getAsJsonObject("start");
        double sx = st.get("x").getAsDouble() * k;
        double sy = st.get("y").getAsDouble() * k;
        double shDeg = st.has("heading") ? st.get("heading").getAsDouble() : 0.0;

        RobotPath path = new RobotPath(sx, sy, shDeg);
        double curX = sx, curY = sy, curH = Math.toRadians(shDeg);
        path.points.add(new RobotPath.Point(curX, curY, curH));

        int segIndex = 0;
        for (JsonElement el : root.getAsJsonArray("segments")) {
            JsonObject seg = el.getAsJsonObject();
            String type = seg.get("type").getAsString();

            // punti di controllo della Bezier: partenza, controlli, arrivo
            List<double[]> ctrl = new ArrayList<>();
            ctrl.add(new double[]{curX, curY});
            if (type.equals("curve")) {
                if (seg.has("controls")) {
                    for (JsonElement c : seg.getAsJsonArray("controls")) {
                        JsonArray a = c.getAsJsonArray();
                        ctrl.add(new double[]{a.get(0).getAsDouble() * k, a.get(1).getAsDouble() * k});
                    }
                }
            } else if (!type.equals("line")) {
                throw new IllegalArgumentException("Segmento " + segIndex + ": tipo sconosciuto '" + type + "'");
            }
            JsonArray to = seg.getAsJsonArray("to");
            ctrl.add(new double[]{to.get(0).getAsDouble() * k, to.get(1).getAsDouble() * k});

            // modalita' heading
            boolean tangent = false;
            Double target = null;
            JsonElement h = seg.get("heading");
            if (h != null && h.isJsonPrimitive()) {
                if (h.getAsJsonPrimitive().isNumber()) {
                    target = Math.toRadians(h.getAsDouble());
                } else if ("tangent".equals(h.getAsString())) {
                    tangent = true;
                } else {
                    throw new IllegalArgumentException("Segmento " + segIndex + ": heading non valido '" + h.getAsString() + "'");
                }
            }

            // campionamento
            double poly = 0;
            for (int i = 1; i < ctrl.size(); i++) {
                poly += Math.hypot(ctrl.get(i)[0] - ctrl.get(i - 1)[0], ctrl.get(i)[1] - ctrl.get(i - 1)[1]);
            }
            int n = (int) Math.max(2, Math.min(300, Math.ceil(poly / SAMPLE_STEP_CM)));

            double[] xs = new double[n + 1], ys = new double[n + 1], cum = new double[n + 1];
            for (int i = 0; i <= n; i++) {
                double[] p = bezier(ctrl, (double) i / n);
                xs[i] = p[0];
                ys[i] = p[1];
                if (i > 0) cum[i] = cum[i - 1] + Math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
            }
            double total = cum[n];

            double startH = curH;
            double prevH = curH;
            for (int i = 1; i <= n; i++) {
                double hd;
                if (tangent) {
                    double dx = xs[i] - xs[i - 1], dy = ys[i] - ys[i - 1];
                    // heading 0 = +Y, antiorario positivo -> direzione (-sin, cos)
                    hd = Math.hypot(dx, dy) < 1e-9 ? prevH : Math.atan2(-dx, dy);
                } else if (target != null) {
                    double f = total > 1e-9 ? cum[i] / total : 1.0;
                    hd = startH + RobotPath.wrap(target - startH) * f;
                } else {
                    hd = startH;
                }
                hd = RobotPath.wrap(hd);
                prevH = hd;
                path.points.add(new RobotPath.Point(xs[i], ys[i], hd));
            }

            curX = xs[n];
            curY = ys[n];
            curH = prevH;
            segIndex++;
        }

        path.finish();
        return path;
    }

    private static double unitFactor(JsonObject root) {
        String u = root.has("units") ? root.get("units").getAsString() : "in";
        if (u.equals("in")) return 2.54;
        if (u.equals("cm")) return 1.0;
        throw new IllegalArgumentException("units deve essere 'in' o 'cm'");
    }

    /** De Casteljau per un numero qualsiasi di punti di controllo. */
    private static double[] bezier(List<double[]> c, double t) {
        int m = c.size();
        double[] bx = new double[m], by = new double[m];
        for (int i = 0; i < m; i++) {
            bx[i] = c.get(i)[0];
            by[i] = c.get(i)[1];
        }
        for (int r = 1; r < m; r++) {
            for (int i = 0; i < m - r; i++) {
                bx[i] = (1 - t) * bx[i] + t * bx[i + 1];
                by[i] = (1 - t) * by[i] + t * by[i + 1];
            }
        }
        return new double[]{bx[0], by[0]};
    }
}
