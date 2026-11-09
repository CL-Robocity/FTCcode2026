package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardware;

/**
 * Meccanismi comandati dal JSON: intake, flywheel e "sparo".
 *
 * Tutti i valori qui sotto sono NaN = "non ancora impostato": finche' restano NaN
 * il meccanismo corrispondente NON viene toccato (nessun movimento a sorpresa).
 * Compilali con i valori che usi nel teleop e controlla in telemetria missingConfig().
 */
public class RobotActions {

    // ===== DA CONFIGURARE =====
    public static final double INTAKE_POWER = Double.NaN;            // FrontIntake, es. 1.0 (o -1.0)
    public static final double ROLLER_POWER = Double.NaN;            // IntakeRoller con intake attivo
    public static final double FLYWHEEL_TICKS_PER_SEC = Double.NaN;  // velocita' target dei due flywheel
    public static final double BALLSTOP_CLOSED = Double.NaN;         // posizione servo ballstop chiuso (0..1)
    public static final double BALLSTOP_OPEN = Double.NaN;           // posizione servo ballstop aperto (0..1)
    public static final double FEED_ROLLER_POWER = Double.NaN;       // IntakeRoller mentre si spara

    private final RobotHardware robot;
    private boolean intakeOn = false;
    private boolean shooting = false;

    public RobotActions(RobotHardware robot) {
        this.robot = robot;
        if (!Double.isNaN(FLYWHEEL_TICKS_PER_SEC)) {
            robot.TopFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DownFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setIntake(boolean on) {
        intakeOn = on;
        apply();
    }

    public void setFlywheel(boolean on) {
        if (Double.isNaN(FLYWHEEL_TICKS_PER_SEC)) return;
        double v = on ? FLYWHEEL_TICKS_PER_SEC : 0.0;
        robot.TopFlyWheel.setVelocity(v);
        robot.DownFlyWheel.setVelocity(v);
    }

    /** Alimenta le palline verso il flywheel (ballstop aperto + roller). */
    public void setShooting(boolean on) {
        shooting = on;
        apply();
    }

    public void stopAll() {
        intakeOn = false;
        shooting = false;
        apply();
        setFlywheel(false);
    }

    private void apply() {
        if (!Double.isNaN(INTAKE_POWER)) {
            robot.FrontIntake.setPower(intakeOn ? INTAKE_POWER : 0.0);
        }
        if (!Double.isNaN(ROLLER_POWER) || !Double.isNaN(FEED_ROLLER_POWER)) {
            double roller = 0.0;
            if (shooting && !Double.isNaN(FEED_ROLLER_POWER)) roller = FEED_ROLLER_POWER;
            else if (intakeOn && !Double.isNaN(ROLLER_POWER)) roller = ROLLER_POWER;
            robot.IntakeRoller.setPower(roller);
        }
        if (!Double.isNaN(BALLSTOP_OPEN) && !Double.isNaN(BALLSTOP_CLOSED)) {
            robot.BallStopServo.setPosition(shooting ? BALLSTOP_OPEN : BALLSTOP_CLOSED);
        }
    }

    /** Elenco dei valori ancora da impostare (vuoto = tutto configurato). */
    public String missingConfig() {
        StringBuilder sb = new StringBuilder();
        if (Double.isNaN(INTAKE_POWER)) sb.append("INTAKE_POWER ");
        if (Double.isNaN(ROLLER_POWER)) sb.append("ROLLER_POWER ");
        if (Double.isNaN(FLYWHEEL_TICKS_PER_SEC)) sb.append("FLYWHEEL_TICKS_PER_SEC ");
        if (Double.isNaN(BALLSTOP_CLOSED)) sb.append("BALLSTOP_CLOSED ");
        if (Double.isNaN(BALLSTOP_OPEN)) sb.append("BALLSTOP_OPEN ");
        if (Double.isNaN(FEED_ROLLER_POWER)) sb.append("FEED_ROLLER_POWER ");
        return sb.toString().trim();
    }
}
