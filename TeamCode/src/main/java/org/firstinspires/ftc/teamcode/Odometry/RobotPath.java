
package org.firstinspires.ftc.teamcode.Odometry;

import java.util.ArrayList;
import java.util.List;

/**
 * Percorso letto dal JSON dell'HUD, già convertito nella convenzione del robot.
 *
 * Unità: cm.
 * Heading: radianti.
 * 0 = robot rivolto verso +Y, positivo antiorario.
 */
public class RobotPath {

    public enum ActionType {
        MOVE, WAIT, SHOOT_STOP, SHOOT_MOVE
    }

    public static class Point {

        public final double x, y, heading;

        // Distanza percorsa dall'inizio del segmento (cm)
        public double s;

        public Point(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public static class Segment {

        public final List<Point> points = new ArrayList<>();

        // Profilo di velocità (cm/s e cm/s²)
        public double vStart = 0;
        public double vMin = 25;
        public double vMax = 80;

        public double accel = 120;
        public double decel = 200;

        // Frazione della lunghezza alla quale inizia la decelerazione
        public double brakePoint = 0.75;

        // Azione associata al segmento
        public ActionType action = ActionType.MOVE;
        public double actionSeconds = 0;

        // Meccanismi
        public boolean intake = false;
        public boolean flywheel = false;

        // Orientamento del robot, indipendente dalla direzione di movimento
        public double headingStart;
        public double headingEnd;

        // Velocità angolare richiesta (gradi/s)
        public double rotationSpeedDegS = 180;

        /**
         * Calcola la distanza cumulativa lungo il percorso.
         */
        public void finish() {

            double acc = 0;

            for (int i = 0; i < points.size(); i++) {

                Point p = points.get(i);

                if (i > 0) {
                    Point previous = points.get(i - 1);

                    acc += Math.hypot(
                            p.x - previous.x,
                            p.y - previous.y
                    );
                }

                p.s = acc;
            }
        }

        public Point last() {
            return points.get(points.size() - 1);
        }

        public double length() {
            return last().s;
        }

        /**
         * Restituisce il punto corrispondente alla distanza s
         * dall'inizio del segmento.
         */
        public Point pointAt(double s) {

            if (s <= 0) {
                return points.get(0);
            }

            if (s >= length()) {
                return last();
            }

            int lo = 0;
            int hi = points.size() - 1;

            // Ricerca binaria del tratto contenente s
            while (hi - lo > 1) {

                int mid = (lo + hi) / 2;

                if (points.get(mid).s <= s) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }

            Point a = points.get(lo);
            Point b = points.get(hi);

            double span = b.s - a.s;

            double t = span < 1e-9
                    ? 0
                    : (s - a.s) / span;

            Point p = new Point(
                    a.x + (b.x - a.x) * t,
                    a.y + (b.y - a.y) * t,
                    a.heading + wrap(b.heading - a.heading) * t
            );

            p.s = s;

            return p;
        }
    }

    // Posizione e orientamento iniziali
    public final double startX;
    public final double startY;
    public final double startHeadingDeg;

    public double matchDurationSec = 30;

    public final List<Segment> segments = new ArrayList<>();

    public RobotPath(
            double startX,
            double startY,
            double startHeadingDeg
    ) {
        this.startX = startX;
        this.startY = startY;
        this.startHeadingDeg = startHeadingDeg;
    }

    /**
     * Normalizza un angolo nell'intervallo [-PI, PI].
     */
    public static double wrap(double angle) {
        return Math.atan2(
                Math.sin(angle),
                Math.cos(angle)
        );
    }
}
