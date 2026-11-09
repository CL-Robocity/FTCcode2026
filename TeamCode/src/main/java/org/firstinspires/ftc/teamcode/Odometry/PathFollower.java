package org.firstinspires.ftc.teamcode.Odometry;

import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardware;

/**
 * Controller di movimento FTC.
 *
 * - Segue un segmento alla volta.
 * - Insegue il percorso con lookahead.
 * - Gestisce traslazione e rotazione indipendentemente.
 * - Ruota da headingStart a headingEnd durante il movimento.
 * - Completa la rotazione sul posto se necessario.
 * - La sequenza dei segmenti e' gestita da AutoRunner.
 *
 * NON chiama loc.update(): lo fa AutoRunner.
 */
public class PathFollower {

    // ===== SEGNI =====
    // Potenza positiva = avanti
    // Strafe positivo = destra
    // Turn positivo = antiorario

    private static final double FORWARD_SIGN = 1.0;
    private static final double STRAFE_SIGN = 1.0;
    private static final double TURN_SIGN = 1.0;

    // ===== CALIBRAZIONE ROBOT =====

    public double maxSpeedCmS = 100.0;
    public double maxRotSpeedDegS = 300.0;

    // ===== TUNING =====

    public double minPower = 0.12;
    public double minTurnPower = 0.10;

    public double lookaheadCm = 20.0;

    public double kHeading = 1.2;

    // Limite rotazione durante la traslazione
    public double maxTurnWhileDriving = 1.0;

    public double posTolCm = 3.0;
    public double headTolDeg = 5.0;
    public double rotateTolDeg = 2.0;

    public double segmentTimeoutSec = 15.0;
    public double rotateTimeoutSec = 4.0;

    private static final double SEARCH_WINDOW_CM = 40.0;

    private final RobotHardware robot;
    private final PositionFinder loc;

    // ===== SEGMENTO =====

    private RobotPath.Segment seg;

    private int idx;

    private double vCmd;

    private long lastNs;
    private long startNs;

    // Heading richiesto
    private double headingDelta;
    private double headingDirection;
    private double rotationSpeed;
    private double rotationPower;
    // Rotazione sul posto
    private double rotTarget;
    private double rotMaxPower;
    private long rotStartNs;

    // Rotazione finale sul posto
    private boolean finalRotation;
    private long finalRotationStartNs;

    // ===== TELEMETRIA =====

    public double lastEndDist;
    public double lastHeadErrDeg;
    public double lastSpeedCmd;

    public double lastTargetHeadingDeg;
    public double lastProgress;

    public PathFollower(RobotHardware robot, PositionFinder loc) {
        this.robot = robot;
        this.loc = loc;
    }

    // ============================================================
    // INIZIO SEGMENTO
    // ============================================================

    public void startSegment(RobotPath.Segment s) {

        seg = s;

        idx = 0;

        // Parte almeno dalla velocita' minima del JSON,
        // altrimenti con v_start = 0 il robot non si sblocca.
        vCmd = Math.max(s.vStart, s.vMin);

        startNs = System.nanoTime();
        lastNs = startNs;

        finalRotation = false;

        // Rotazione indipendente dal progresso spaziale.
        headingDelta = RobotPath.wrap(
                s.headingEnd - s.headingStart
        );

        headingDirection = Math.signum(headingDelta);

        rotationSpeed = Math.max(0.0, s.rotationSpeedDegS);

        // Potenza corrispondente alla velocita' angolare richiesta.
        rotationPower = rotationSpeed / maxRotSpeedDegS;

        rotationPower = clamp(
                rotationPower,
                minTurnPower,
                1.0
        );

        lastSpeedCmd = vCmd;
    }

    // ============================================================
    // AGGIORNAMENTO SEGMENTO
    // ============================================================

    public boolean updateSegment() {

        if (seg == null) {
            return false;
        }

        long now = System.nanoTime();

        double dt = Math.min(
                (now - lastNs) / 1e9,
                0.1
        );

        lastNs = now;

        double elapsed = (now - startNs) / 1e9;

        double x = loc.getX();
        double y = loc.getY();
        double h = loc.getHeadingRadians();

        // --------------------------------------------------------
        // 1. PROGRESSO SUL PERCORSO
        // --------------------------------------------------------

        double sNow = seg.points.get(idx).s;

        double best = Double.MAX_VALUE;
        int bestIdx = idx;

        for (int i = idx; i < seg.points.size(); i++) {

            RobotPath.Point p = seg.points.get(i);

            if (p.s > sNow + Math.max(SEARCH_WINDOW_CM, lookaheadCm * 2)) {
                break;
            }

            double d = Math.hypot(
                    p.x - x,
                    p.y - y
            );

            if (d < best) {
                best = d;
                bestIdx = i;
            }
        }

        idx = bestIdx;

        double s = seg.points.get(idx).s;

        double length = seg.length();

        double remaining = Math.max(0.0, length - s);

        double f = length > 1e-6
                ? clamp(s / length, 0.0, 1.0)
                : 1.0;

        lastProgress = f * 100.0;

        RobotPath.Point end = seg.last();

        double endDist = Math.hypot(
                end.x - x,
                end.y - y
        );

        lastEndDist = endDist;

        // --------------------------------------------------------
        // 2. GESTIONE HEADING
        // --------------------------------------------------------

        double rotationProgress = Math.min(
                Math.abs(headingDelta),
                Math.toRadians(rotationSpeed) * elapsed
        );

        double desiredHeading =
                seg.headingStart +
                        headingDirection * rotationProgress;

        boolean rotationTargetReached =
                rotationProgress >= Math.abs(headingDelta);

        if (rotationTargetReached) {
            desiredHeading = seg.headingEnd;
        }

        double headingError = RobotPath.wrap(
                desiredHeading - h
        );

        lastTargetHeadingDeg =
                Math.toDegrees(desiredHeading);

        lastHeadErrDeg = Math.toDegrees(
                RobotPath.wrap(seg.headingEnd - h)
        );

        // Feedforward: permette di seguire la velocita'
        // angolare richiesta senza affidarsi solamente al P.
        double turn = headingControl(
                headingError,
                rotationTargetReached
                        ? 0.0
                        : headingPower() * headingDirection
        );

        // --------------------------------------------------------
        // 3. FINE TRASLAZIONE
        // --------------------------------------------------------

        if (finalRotation || endDist < posTolCm) {

            if (!finalRotation) {
                finalRotation = true;
                finalRotationStartNs = now;
            }

            // Una volta arrivati, completiamo la rotazione
            // sul posto verso headingEnd.
            double finalError = RobotPath.wrap(
                    seg.headingEnd - h
            );

            lastHeadErrDeg = Math.toDegrees(finalError);

            if (Math.abs(finalError) < Math.toRadians(rotateTolDeg)) {

                stopMotors();

                seg = null;

                lastSpeedCmd = 0.0;

                return false;
            }

            // Sicurezza contro una rotazione bloccata.
            boolean rotateTimedOut =
                    (now - finalRotationStartNs) / 1e9
                            > rotateTimeoutSec;

            if (rotateTimedOut) {

                stopMotors();

                seg = null;

                return false;
            }

            turn = headingControl(finalError, 0.0);

            drive(0.0, 0.0, turn);

            lastSpeedCmd = 0.0;

            return true;
        }

        // Timeout di sicurezza del segmento.
        boolean timedOut = elapsed > segmentTimeoutSec;

        if (timedOut) {

            stopMotors();

            seg = null;

            return false;
        }

        // --------------------------------------------------------
        // 4. PROFILO DI VELOCITA'
        // --------------------------------------------------------

        double vTarget = profile(s, f, length, endDist);

        // Limitazione fisica dell'accelerazione e decelerazione.
        vCmd = clamp(
                vTarget,
                vCmd - seg.decel * dt,
                vCmd + seg.accel * dt
        );

        lastSpeedCmd = vCmd;

        double power = vCmd / maxSpeedCmS;

        if (power > 0.0) {
            power = Math.max(minPower, power);
        }

        power = clamp(power, 0.0, 1.0);

        // --------------------------------------------------------
        // 5. LOOKAHEAD
        // --------------------------------------------------------

        double targetS = Math.min(
                s + lookaheadCm,
                length
        );

        RobotPath.Point target = seg.pointAt(targetS);

        double dx = target.x - x;
        double dy = target.y - y;

        double dist = Math.hypot(dx, dy);

        double ux = dist > 1e-6 ? dx / dist : 0.0;
        double uy = dist > 1e-6 ? dy / dist : 0.0;

        // --------------------------------------------------------
        // 6. CAMPO -> ROBOT
        // --------------------------------------------------------

        // Avanti = (-sin(h), cos(h))
        // Destra = (cos(h), sin(h))

        // Avanti robot nel campo = ( sin(h),  cos(h)) con 0°=+Y, antiorario
        // Destra robot nel campo = ( cos(h), -sin(h))
        double forwardDirection =
                ux * Math.sin(h) +
                        uy * Math.cos(h);

        double strafeDirection =
                ux * Math.cos(h) -
                        uy * Math.sin(h);

        double forward = forwardDirection * power;
        double strafe = strafeDirection * power;

        // --------------------------------------------------------
        // 7. TRASLAZIONE + ROTAZIONE
        // --------------------------------------------------------

        turn = clamp(
                turn,
                -maxTurnWhileDriving,
                maxTurnWhileDriving
        );

        drive(forward, strafe, turn);

        return true;
    }

    // ============================================================
    // PROFILO VELOCITA'
    // ============================================================

    private double profile(
            double s,
            double progress,
            double length,
            double endDist
    ) {

        double bp = clamp(
                seg.brakePoint,
                0.01,
                0.99
        );

        double brakeDistance = length * bp;

        double vStart = Math.max(0.0, seg.vStart);

        double vMax = Math.max(vStart, seg.vMax);

        double accel = Math.max(0.0, seg.accel);

        double decel = Math.max(0.0, seg.decel);

        // vMin viene dal JSON (v_min). Il minimo di 1.0 evita
        // che un v_min = 0 riporti il blocco a s = 0.
        double vMin = Math.min(
                vMax,
                Math.max(seg.vMin, 1.0)
        );

        // Velocita' raggiunta teoricamente al brake point.
        double vAtBrake = Math.min(
                vMax,
                Math.sqrt(
                        vStart * vStart +
                                2.0 * accel * brakeDistance
                )
        );

        double vTarget;

        if (progress < bp) {

            // Fase di accelerazione.
            vTarget = Math.min(
                    vMax,
                    Math.sqrt(
                            vStart * vStart +
                                    2.0 * accel * s
                    )
            );

            // Evita il blocco a s = 0 con v_start = 0.
            vTarget = Math.max(vMin, vTarget);

        } else {

            // Fase di decelerazione.
            double distanceAfterBrake = Math.max(
                    0.0,
                    s - brakeDistance
            );

            vTarget = Math.sqrt(
                    Math.max(
                            0.0,
                            vAtBrake * vAtBrake -
                                    2.0 * decel * distanceAfterBrake
                    )
            );

            // Scala vMin a 0 negli ultimi centimetri così il robot si ferma
            double endFactor = clamp(endDist / (posTolCm * 3.0), 0.0, 1.0);
            vTarget = Math.max(vMin * endFactor, vTarget);
        }

        return vTarget;
    }

    // ============================================================
    // CONTROLLO HEADING
    // ============================================================

    private double headingPower() {
        return rotationPower;
    }

    /**
     * Feedforward + proporzionale.
     *
     * desiredFeedforward:
     * potenza necessaria per la velocita' angolare richiesta.
     */
    private double headingControl(
            double error,
            double desiredFeedforward
    ) {

        if (Math.abs(error) < Math.toRadians(rotateTolDeg)) {
            return 0.0;
        }

        double correction = kHeading * error;

        double output = desiredFeedforward + correction;

        return clamp(output, -1.0, 1.0);
    }
    // ============================================================
    // ROTAZIONE SUL POSTO
    // ============================================================

    public void startRotation(
            double targetRad,
            double rotSpeedDegS
    ) {

        rotTarget = targetRad;

        rotMaxPower = clamp(
                rotSpeedDegS / maxRotSpeedDegS,
                minTurnPower,
                1.0
        );

        rotStartNs = System.nanoTime();
    }

    public boolean updateRotation() {

        double err = RobotPath.wrap(
                rotTarget - loc.getHeadingRadians()
        );

        boolean timedOut =
                (System.nanoTime() - rotStartNs) / 1e9
                        > rotateTimeoutSec;

        if (Math.abs(err) < Math.toRadians(rotateTolDeg)
                || timedOut) {

            stopMotors();

            return false;
        }

        double turn = Math.signum(err) * clamp(
                Math.abs(kHeading * err),
                minTurnPower,
                rotMaxPower
        );

        drive(0.0, 0.0, turn);

        return true;
    }

    // ============================================================
    // MOTORI
    // ============================================================

    public void stopMotors() {
        drive(0.0, 0.0, 0.0);
    }

    /**
     * Mix mecanum.
     * Turn positivo = antiorario.
     */
    private void drive(
            double forward,
            double strafe,
            double turn
    ) {

        double f = forward * FORWARD_SIGN;
        double s = strafe * STRAFE_SIGN;
        double t = turn * TURN_SIGN;

        double lf = f + s - t;
        double lb = f - s - t;
        double rf = f - s + t;
        double rb = f + s + t;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(lf),
                        Math.max(
                                Math.abs(lb),
                                Math.max(
                                        Math.abs(rf),
                                        Math.abs(rb)
                                )
                        )
                )
        );

        robot.lfD.setPower(lf / max);
        robot.lbD.setPower(lb / max);
        robot.rfD.setPower(rf / max);
        robot.rbD.setPower(rb / max);
    }

    private static double clamp(
            double v,
            double lo,
            double hi
    ) {
        return Math.max(lo, Math.min(hi, v));
    }
}