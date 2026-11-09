
package org.firstinspires.ftc.teamcode.Odometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Esegue il percorso segmento per segmento.
 *
 * Sequenza:
 * 1. Allineamento all'heading iniziale del segmento.
 * 2. Movimento con rotazione simultanea.
 * 3. Eventuale attesa o azione.
 * 4. Passaggio al segmento successivo.
 *
 * Le azioni dei meccanismi vengono solamente riportate
 * nella telemetry: non vengono ancora attivati.
 */
public class AutoRunner {

    private enum State {
        ROTATE, DRIVE, HOLD, DONE
    }

    private final PositionFinder loc;
    private final PathFollower follower;
    private final RobotActions actions;
    private final Telemetry telemetry;

    private RobotPath path;
    private RobotPath.Segment seg;

    private int segIndex;

    private State state = State.DONE;

    private long matchStartNs;
    private long segStartNs;
    private long holdUntilNs;

    private double prevEndHeading;

    private boolean shootMoveOn;

    public AutoRunner(
            PositionFinder loc,
            PathFollower follower,
            RobotActions actions,
            Telemetry telemetry
    ) {
        this.loc = loc;
        this.follower = follower;
        this.actions = actions;
        this.telemetry = telemetry;
    }

    /**
     * Avvia l'esecuzione di un percorso.
     */
    public void start(RobotPath p) {

        path = p;

        segIndex = 0;

        prevEndHeading = Math.toRadians(p.startHeadingDeg);

        matchStartNs = System.nanoTime();

        shootMoveOn = false;

        follower.stopMotors();

        telemetry.addData("Auto", "Avvio percorso");
        telemetry.addData("Segmenti", path.segments.size());

        beginSegment();
    }

    /**
     * Un ciclo di esecuzione.
     *
     * Ritorna true finché il percorso è in corso.
     */
    public boolean update() {

        if (state == State.DONE) {
            return false;
        }

        loc.update();

        // Controllo durata massima dell'autonomia
        if (elapsedSec() >= path.matchDurationSec) {

            telemetry.addData("Auto", "Tempo massimo raggiunto");

            stop();

            return false;
        }

        switch (state) {

            // -----------------------------------------
            // ROTAZIONE PRIMA DEL SEGMENTO
            // -----------------------------------------

            case ROTATE:

                if (!follower.updateRotation()) {

                    telemetry.addData(
                            "Auto",
                            "Allineamento completato"
                    );

                    enterDrive();
                }

                break;

            // -----------------------------------------
            // MOVIMENTO
            // -----------------------------------------

            case DRIVE:

                // Termine dell'azione SHOOT_MOVE
                if (shootMoveOn &&
                        secondsSince(segStartNs) >= seg.actionSeconds) {

                    telemetry.addData(
                            "Action",
                            "SHOOT_MOVE terminata"
                    );

                    shootMoveOn = false;
                }

                if (!follower.updateSegment()) {
                    onSegmentFinished();
                }

                break;

            // -----------------------------------------
            // ATTESA
            // -----------------------------------------

            case HOLD:

                follower.stopMotors();

                if (System.nanoTime() >= holdUntilNs) {

                    telemetry.addData(
                            "Action",
                            "Attesa terminata"
                    );

                    nextSegment();
                }

                break;

            default:
                break;
        }

        return state != State.DONE;
    }

    /**
     * Ferma tutto.
     */
    public void stop() {

        follower.stopMotors();

        actions.stopAll();

        shootMoveOn = false;

        state = State.DONE;

        telemetry.addData("Auto", "Terminata");
    }

    public double elapsedSec() {

        return (System.nanoTime() - matchStartNs) / 1e9;
    }

    public String getStatus() {

        int n = path == null ? 0 : path.segments.size();

        return String.format(
                "seg %d/%d %s",
                Math.min(segIndex + 1, n),
                n,
                state
        );
    }

    // =================================================
    // INTERNAL METHODS
    // =================================================

    /**
     * Prepara il segmento successivo.
     */
    private void beginSegment() {

        if (segIndex >= path.segments.size()) {

            stop();

            return;
        }

        seg = path.segments.get(segIndex);

        telemetry.addData(
                "Segmento",
                "%d / %d",
                segIndex + 1,
                path.segments.size()
        );

        telemetry.addData(
                "Intake richiesto",
                seg.intake
        );

        telemetry.addData(
                "Flywheel richiesto",
                seg.flywheel
        );

        telemetry.addData(
                "Heading iniziale",
                Math.toDegrees(seg.headingStart)
        );

        telemetry.addData(
                "Heading finale",
                Math.toDegrees(seg.headingEnd)
        );

        // Confronto tra heading finale precedente
        // e heading iniziale del segmento corrente.
        double delta = RobotPath.wrap(
                seg.headingStart - prevEndHeading
        );

        if (Math.abs(delta) > Math.toRadians(1.0)) {

            telemetry.addData(
                    "Auto",
                    "Allineamento prima del segmento"
            );

            follower.startRotation(
                    seg.headingStart,
                    seg.rotationSpeedDegS
            );

            state = State.ROTATE;

        } else {

            enterDrive();
        }
    }

    /**
     * Inizia il movimento del segmento.
     */
    private void enterDrive() {

        follower.startSegment(seg);

        segStartNs = System.nanoTime();

        shootMoveOn =
                seg.action == RobotPath.ActionType.SHOOT_MOVE
                        && seg.actionSeconds > 0;

        if (shootMoveOn) {

            telemetry.addData(
                    "Action",
                    "SHOOT_MOVE richiesta per %.2f secondi",
                    seg.actionSeconds
            );
        }

        state = State.DRIVE;
    }

    /**
     * Chiamato quando il segmento è terminato.
     */
    private void onSegmentFinished() {

        // Il PathFollower ha completato il segmento
        // e ha raggiunto l'heading finale.
        prevEndHeading = seg.headingEnd;

        if (shootMoveOn) {

            telemetry.addData(
                    "Action",
                    "SHOOT_MOVE terminata per fine segmento"
            );

            shootMoveOn = false;
        }

        switch (seg.action) {

            case WAIT:

                telemetry.addData(
                        "Action",
                        "WAIT richiesta: %.2f secondi",
                        seg.actionSeconds
                );

                startHold(seg.actionSeconds);

                break;

            case SHOOT_STOP:

                telemetry.addData(
                        "Action",
                        "SHOOT_STOP richiesta: %.2f secondi",
                        seg.actionSeconds
                );

                startHold(seg.actionSeconds);

                break;

            case SHOOT_MOVE:
            case MOVE:

                nextSegment();

                break;
        }
    }

    /**
     * Avvia un'attesa a robot fermo.
     */
    private void startHold(double seconds) {

        follower.stopMotors();

        if (seconds <= 0) {

            nextSegment();

            return;
        }

        holdUntilNs = System.nanoTime()
                + (long) (seconds * 1e9);

        state = State.HOLD;
    }

    /**
     * Passa al segmento successivo.
     */
    private void nextSegment() {

        segIndex++;

        beginSegment();
    }

    private double secondsSince(long ns) {

        return (System.nanoTime() - ns) / 1e9;
    }
}
