package org.firstinspires.ftc.teamcode.Odometry;

import java.util.ArrayList;
import java.util.List;

/**
 * Percorso campionato in punti ravvicinati.
 * Unita': cm. Heading: radianti, 0 = robot rivolto verso +Y, positivo antiorario
 * (stessa convenzione del PositionFinder).
 */
public class RobotPath {

    public static class Point {
        public final double x, y, heading;
        public double s; // lunghezza d'arco dall'inizio del percorso (cm)

        public Point(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public final double startX, startY, startHeadingDeg;
    public final List<Point> points = new ArrayList<>();

    public RobotPath(double startX, double startY, double startHeadingDeg) {
        this.startX = startX;
        this.startY = startY;
        this.startHeadingDeg = startHeadingDeg;
    }

    /** Da chiamare dopo aver aggiunto tutti i punti: calcola la lunghezza d'arco. */
    public void finish() {
        double acc = 0;
        for (int i = 0; i < points.size(); i++) {
            Point p = points.get(i);
            if (i > 0) {
                Point q = points.get(i - 1);
                acc += Math.hypot(p.x - q.x, p.y - q.y);
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

    /** Punto interpolato alla distanza s (cm) dall'inizio. */
    public Point pointAt(double s) {
        if (s <= 0) return points.get(0);
        if (s >= length()) return last();

        int lo = 0, hi = points.size() - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            if (points.get(mid).s <= s) lo = mid;
            else hi = mid;
        }
        Point a = points.get(lo), b = points.get(hi);
        double span = b.s - a.s;
        double t = span < 1e-9 ? 0 : (s - a.s) / span;
        Point p = new Point(
                a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t,
                a.heading + wrap(b.heading - a.heading) * t);
        p.s = s;
        return p;
    }

    public static double wrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
