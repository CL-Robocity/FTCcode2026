
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
 * Legge il JSON esportato dall'HUD e lo trasforma in un RobotPath.
 *
 * Coordinate interne: centimetri.
 * Velocità interne: cm/s.
 * Accelerazioni interne: cm/s².
 *
 * Heading:
 * HUD: 0 = +X
 * Robot: 0 = +Y
 */
public class PathLoader {

    public static final String FOLDER = "/sdcard/FIRST/paths/";

    private static final double SAMPLE_STEP_CM = 2.0;

    public static RobotPath load(String fileName) throws IOException {

        StringBuilder sb = new StringBuilder();

        File file = new File(FOLDER + fileName);

        try (BufferedReader r = new BufferedReader(new FileReader(file))) {

            String line;

            while ((line = r.readLine()) != null) {
                sb.append(line).append('\n');
            }
        }

        return parse(sb.toString());
    }

    public static RobotPath parse(String json) {

        JsonObject root = new JsonParser()
                .parse(json)
                .getAsJsonObject();

        double k = unitFactor(root);

        // -------------------------------------------------
        // START
        // -------------------------------------------------

        JsonObject st = root.getAsJsonObject("start");

        double sx = st.get("x").getAsDouble() * k;
        double sy = st.get("y").getAsDouble() * k;

        double startHudDeg = num(st, "heading", 90.0);

        RobotPath path = new RobotPath(
                sx,
                sy,
                hudToRobotDeg(startHudDeg)
        );

        path.matchDurationSec = num(
                root,
                "match_duration",
                30.0
        );

        double curX = sx;
        double curY = sy;

        double curHeading = Math.toRadians(path.startHeadingDeg);

        // -------------------------------------------------
        // SEGMENTS
        // -------------------------------------------------

        JsonArray segments = root.getAsJsonArray("segments");

        int segIndex = 0;

        for (JsonElement el : segments) {

            JsonObject o = el.getAsJsonObject();

            RobotPath.Segment seg = new RobotPath.Segment();

            String type = o.get("type").getAsString();

            double[] from = {curX, curY};

            double[] to = readPoint(o.get("to"), k);

            // -------------------------------------------------
            // BEZIER CONTROL POINTS
            // -------------------------------------------------

            double[] p1 = from;
            double[] p2 = to;

            if (type.equals("curve")) {

                List<double[]> ctrl = new ArrayList<>();

                if (o.has("controls")) {

                    for (JsonElement c : o.getAsJsonArray("controls")) {
                        ctrl.add(readPoint(c, k));
                    }
                }

                if (ctrl.size() == 1) {

                    p1 = ctrl.get(0);
                    p2 = ctrl.get(0);

                } else if (ctrl.size() >= 2) {

                    p1 = ctrl.get(0);
                    p2 = ctrl.get(ctrl.size() - 1);
                }

            } else if (!type.equals("line")) {

                throw new IllegalArgumentException(
                        "Segmento " + segIndex +
                                ": tipo sconosciuto '" + type + "'"
                );
            }

            // -------------------------------------------------
            // ROBOT HEADING
            // -------------------------------------------------

            JsonObject rh = o.has("robot_heading")
                    ? o.getAsJsonObject("robot_heading")
                    : null;

            double hs = rh != null && rh.has("start")
                    ? Math.toRadians(
                    hudToRobotDeg(rh.get("start").getAsDouble())
            )
                    : curHeading;

            double he = rh != null && rh.has("end")
                    ? Math.toRadians(
                    hudToRobotDeg(rh.get("end").getAsDouble())
            )
                    : hs;

            seg.headingStart = hs;
            seg.headingEnd = he;

            seg.rotationSpeedDegS = num(
                    rh,
                    "rotation_speed",
                    180.0
            );

            // -------------------------------------------------
            // SPEED PROFILE
            // -------------------------------------------------

            JsonObject sp = o.has("speed")
                    ? o.getAsJsonObject("speed")
                    : null;

            seg.vStart = num(sp, "v_start", 0.0) * k;

            seg.vMin = num(sp, "v_min", 25.0) * k;

            seg.vMax = num(sp, "v_max", 80.0) * k;

            seg.accel = num(sp, "accel", 120.0) * k;

            seg.decel = num(sp, "decel", 200.0) * k;

            seg.brakePoint = num(
                    sp,
                    "brake_point",
                    0.75
            );

            // -------------------------------------------------
            // ACTIONS
            // -------------------------------------------------

            readAction(
                    o.get("action"),
                    seg,
                    segIndex
            );

            seg.intake = o.has("intake")
                    && o.get("intake").getAsBoolean();

            seg.flywheel = o.has("flywheel")
                    && o.get("flywheel").getAsBoolean();

            // -------------------------------------------------
            // BEZIER SAMPLING
            // -------------------------------------------------

            double poly =
                    Math.hypot(p1[0] - from[0], p1[1] - from[1])
                            + Math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                            + Math.hypot(to[0] - p2[0], to[1] - p2[1]);

            int n = (int) Math.max(
                    4,
                    Math.min(
                            400,
                            Math.ceil(poly / SAMPLE_STEP_CM)
                    )
            );

            double[] xs = new double[n + 1];
            double[] ys = new double[n + 1];
            double[] cum = new double[n + 1];

            for (int i = 0; i <= n; i++) {

                double t = (double) i / n;

                double[] p = cubic(
                        from,
                        p1,
                        p2,
                        to,
                        t
                );

                xs[i] = p[0];
                ys[i] = p[1];

                if (i > 0) {

                    cum[i] = cum[i - 1] + Math.hypot(
                            xs[i] - xs[i - 1],
                            ys[i] - ys[i - 1]
                    );
                }
            }

            double total = cum[n];

            for (int i = 0; i <= n; i++) {

                double f = total > 1e-9
                        ? cum[i] / total
                        : 1.0;

                double hd = RobotPath.wrap(
                        hs + RobotPath.wrap(he - hs) * f
                );

                seg.points.add(
                        new RobotPath.Point(
                                xs[i],
                                ys[i],
                                hd
                        )
                );
            }

            seg.finish();

            path.segments.add(seg);

            // -------------------------------------------------
            // NEXT SEGMENT START
            // -------------------------------------------------

            curX = to[0];
            curY = to[1];

            curHeading = he;

            segIndex++;
        }

        return path;
    }

    // -----------------------------------------------------
    // HELPERS
    // -----------------------------------------------------

    private static void readAction(
            JsonElement a,
            RobotPath.Segment seg,
            int segIndex
    ) {

        if (a == null) return;

        if (a.isJsonObject()) {

            JsonObject o = a.getAsJsonObject();

            if (o.has("wait")) {

                seg.action = RobotPath.ActionType.WAIT;
                seg.actionSeconds = o.get("wait").getAsDouble();

            } else if (o.has("shoot_stop")) {

                seg.action = RobotPath.ActionType.SHOOT_STOP;
                seg.actionSeconds = o.get("shoot_stop").getAsDouble();

            } else if (o.has("shoot_move")) {

                seg.action = RobotPath.ActionType.SHOOT_MOVE;
                seg.actionSeconds = o.get("shoot_move").getAsDouble();

            } else {

                throw new IllegalArgumentException(
                        "Segmento " + segIndex + ": action sconosciuta"
                );
            }

        } else if (!"move".equals(a.getAsString())) {

            throw new IllegalArgumentException(
                    "Segmento " + segIndex +
                            ": action '" + a.getAsString() + "' non valida"
            );
        }
    }

    private static double[] readPoint(JsonElement e, double k) {

        if (e.isJsonArray()) {

            JsonArray a = e.getAsJsonArray();

            return new double[]{
                    a.get(0).getAsDouble() * k,
                    a.get(1).getAsDouble() * k
            };
        }

        JsonObject o = e.getAsJsonObject();

        return new double[]{
                o.get("x").getAsDouble() * k,
                o.get("y").getAsDouble() * k
        };
    }

    private static double num(
            JsonObject o,
            String key,
            double def
    ) {

        return (o != null && o.has(key))
                ? o.get(key).getAsDouble()
                : def;
    }

    /**
     * HUD: 0 = +X
     * Robot: 0 = +Y
     */
    private static double hudToRobotDeg(double hudDeg) {
        return hudDeg - 90.0;
    }

    /**
     * Conversione delle unità in centimetri.
     */
    private static double unitFactor(JsonObject root) {

        String u = root.has("units")
                ? root.get("units").getAsString()
                : "cm";

        if (u.equals("cm")) return 1.0;
        if (u.equals("mm")) return 0.1;
        if (u.equals("in")) return 2.54;

        throw new IllegalArgumentException(
                "units deve essere 'cm', 'mm' o 'in'"
        );
    }

    /**
     * Bezier cubica.
     */
    private static double[] cubic(
            double[] a,
            double[] b,
            double[] c,
            double[] d,
            double t
    ) {

        double u = 1 - t;

        return new double[]{

                u * u * u * a[0]
                        + 3 * u * u * t * b[0]
                        + 3 * u * t * t * c[0]
                        + t * t * t * d[0],

                u * u * u * a[1]
                        + 3 * u * u * t * b[1]
                        + 3 * u * t * t * c[1]
                        + t * t * t * d[1]
        };
    }
}
