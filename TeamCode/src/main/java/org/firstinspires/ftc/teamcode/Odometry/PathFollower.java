package org.firstinspires.ftc.teamcode.Odometry;

import org.firstinspires.ftc.teamcode.Odometry.PositionFinder;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardware;

import java.util.function.BooleanSupplier;

/**
 * Segue un RobotPath usando il PositionFinder.
 * Idea: trova il punto del percorso piu' vicino, punta a un punto "lookahead" piu' avanti,
 * rallenta alla fine, e corregge l'heading con un P.
 */
public class PathFollower {

    // ===== SEGNI (da verificare sul robot, vedi note) =====
    // Con potenza positiva il robot deve andare avanti, strafe positivo = destra,
    // turn positivo = antiorario (come l'heading).
    private static final double FORWARD_SIGN = 1.0;
    private static final double STRAFE_SIGN = 1.0;
    private static final double TURN_SIGN = 1.0;

    // ===== TUNING =====
    public double maxPower = 0.6;
    public double minPower = 0.12;     // potenza minima per vincere l'attrito
    public double lookaheadCm = 20.0;
    public double kPos = 0.02;         // potenza per cm rimasto
    public double kHeading = 1.2;      // potenza per radiante di errore
    public double maxTurn = 0.5;
    public double posTolCm = 2.0;
    public double headTolDeg = 3.0;
    public double timeoutSec = 30.0;

    private static final double SEARCH_WINDOW_CM = 40.0;

    private final RobotHardware robot;
    private final PositionFinder loc;

    private RobotPath path;
    private int idx;
    private boolean running;
    private long startNs;

    // per telemetria
    public double lastEndDist, lastHeadErrDeg;

    public PathFollower(RobotHardware robot, PositionFinder loc) {
        this.robot = robot;
        this.loc = loc;
    }

    public void start(RobotPath p) {
        path = p;
        idx = 0;
        running = true;
        startNs = System.nanoTime();
    }

    /** Un ciclo di controllo. Ritorna true finche' il percorso e' in corso. */
    public boolean update() {
        if (!running) return false;

        loc.update();
        double x = loc.getX(), y = loc.getY(), h = loc.getHeadingRadians();

        // progresso lungo il percorso: punto piu' vicino nella finestra in avanti
        double sNow = path.points.get(idx).s;
        double best = Double.MAX_VALUE;
        int bestIdx = idx;
        for (int i = idx; i < path.points.size(); i++) {
            RobotPath.Point p = path.points.get(i);
            if (p.s > sNow + SEARCH_WINDOW_CM) break;
            double d = Math.hypot(p.x - x, p.y - y);
            if (d < best) {
                best = d;
                bestIdx = i;
            }
        }
        idx = bestIdx;

        double s = path.points.get(idx).s;
        double remaining = path.length() - s;
        RobotPath.Point end = path.last();
        double endDist = Math.hypot(end.x - x, end.y - y);
        double endHeadErr = RobotPath.wrap(end.heading - h);
        lastEndDist = endDist;
        lastHeadErrDeg = Math.toDegrees(endHeadErr);

        boolean timedOut = (System.nanoTime() - startNs) / 1e9 > timeoutSec;
        if ((endDist < posTolCm && Math.abs(endHeadErr) < Math.toRadians(headTolDeg)) || timedOut) {
            stop();
            return false;
        }

        // punto da inseguire
        RobotPath.Point target = path.pointAt(Math.min(s + lookaheadCm, path.length()));
        double dx = target.x - x, dy = target.y - y;
        double dist = Math.hypot(dx, dy);

        double power = 0;
        if (!(endDist < posTolCm && remaining < posTolCm) && dist > 1e-6) {
            power = clamp(kPos * Math.max(remaining, endDist), minPower, maxPower);
        }
        double ux = dist > 1e-6 ? dx / dist : 0;
        double uy = dist > 1e-6 ? dy / dist : 0;

        // da coordinate campo a coordinate robot (avanti = (-sin, cos), destra = (cos, sin))
        double forward = (ux * -Math.sin(h) + uy * Math.cos(h)) * power;
        double strafe = (ux * Math.cos(h) + uy * Math.sin(h)) * power;

        double turn = clamp(kHeading * RobotPath.wrap(target.heading - h), -maxTurn, maxTurn);

        drive(forward, strafe, turn);
        return true;
    }

    public void stop() {
        running = false;
        drive(0, 0, 0);
    }

    /** Versione bloccante, per LinearOpMode: follower.run(path, this::opModeIsActive); */
    public void run(RobotPath p, BooleanSupplier active) {
        start(p);
        while (active.getAsBoolean() && update()) {
            // il ciclo gira alla massima velocita' possibile, serve per l'odometria
        }
        stop();
    }

    // Stesso mix mecanum di StandardMvmnt, con turn positivo = antiorario
    private void drive(double forward, double strafe, double turn) {
        double f = forward * FORWARD_SIGN;
        double s = strafe * STRAFE_SIGN;
        double t = turn * TURN_SIGN;

        double lf = f + s - t;
        double lb = f - s - t;
        double rf = f - s + t;
        double rb = f + s + t;

        double max = Math.max(1, Math.max(Math.abs(lf), Math.max(Math.abs(lb),
                Math.max(Math.abs(rf), Math.abs(rb)))));

        robot.lfD.setPower(lf / max);
        robot.lbD.setPower(lb / max);
        robot.rfD.setPower(rf / max);
        robot.rbD.setPower(rb / max);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
