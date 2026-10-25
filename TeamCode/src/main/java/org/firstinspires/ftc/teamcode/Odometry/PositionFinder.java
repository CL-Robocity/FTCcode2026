package org.firstinspires.ftc.teamcode.Odometry;

import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Odometria con 2 dead wheels + IMU.
 * Unita': cm. Heading: 0 = robot verso +Y, positivo antiorario (come l'IMU FTC).
 */
public class PositionFinder {

    // ===== PARAMETRI DA CALIBRARE =====

    // Tick dell'encoder necessari per percorrere 1 cm
    private static final double TICKS_PER_CM = 100.0;

    // Posizione della ruota parallela rispetto al centro (cm):
    // positiva se si trova a destra del centro del robot
    private static final double PARALLEL_OFFSET = 0.0;

    // Posizione della ruota perpendicolare rispetto al centro (cm):
    // positiva se si trova verso la parte anteriore del robot
    private static final double PERPENDICULAR_OFFSET = 0.0;

    // Segno degli encoder: +1 oppure -1 se la ruota conta al contrario
    // (parallela: avanti = positivo, perpendicolare: destra = positivo)
    private static final double PARALLEL_DIR = 1.0;
    private static final double PERPENDICULAR_DIR = 1.0;


    // ===== HARDWARE E POSIZIONE =====

    private final RobotHardware Robot;

    private double x;
    private double y;
    private double direction; // gradi

    private int lastParallelTicks;
    private int lastPerpendicularTicks;

    private double lastHeading;     // radianti
    private double headingOffset;   // converte l'yaw dell'IMU nell'heading di campo


    // ===== COSTRUTTORI =====

    public PositionFinder(RobotHardware robot, double initialX, double initialY) {
        this(robot, initialX, initialY, 0.0);
    }

    public PositionFinder(
            RobotHardware robot,
            double initialX,
            double initialY,
            double initialHeadingDeg
    ) {
        this.Robot = robot;

        // Coordinate iniziali assolute rispetto all'angolo (0,0)
        this.x = initialX;
        this.y = initialY;

        // Memorizza le letture iniziali degli encoder
        lastParallelTicks = Robot.odoParallel.getCurrentPosition();
        lastPerpendicularTicks = Robot.odoPerp.getCurrentPosition();

        // L'heading di campo parte da initialHeadingDeg, qualunque sia l'yaw dell'IMU ora
        double startHeading = Math.toRadians(initialHeadingDeg);
        headingOffset = startHeading - getRawHeading();
        lastHeading = normalizeAngle(startHeading);
        direction = Math.toDegrees(lastHeading);
    }


    // ===== LETTURA IMU =====

    private double getRawHeading() {
        return Robot.imu
                .getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
    }


    // ===== AGGIORNAMENTO POSIZIONE =====

    public void update() {

        // Lettura encoder attuali
        int currentParallelTicks = Robot.odoParallel.getCurrentPosition();
        int currentPerpendicularTicks = Robot.odoPerp.getCurrentPosition();

        // Differenza rispetto al ciclo precedente
        int deltaParallelTicks = currentParallelTicks - lastParallelTicks;
        int deltaPerpendicularTicks = currentPerpendicularTicks - lastPerpendicularTicks;

        lastParallelTicks = currentParallelTicks;
        lastPerpendicularTicks = currentPerpendicularTicks;

        // Direzione attuale e variazione angolare
        double currentHeading = normalizeAngle(getRawHeading() + headingOffset);
        double deltaHeading = normalizeAngle(currentHeading - lastHeading);

        // Angolo medio durante lo spostamento
        double averageHeading = lastHeading + deltaHeading / 2.0;

        // Conversione da tick a centimetri
        double parallelDistance = PARALLEL_DIR * deltaParallelTicks / TICKS_PER_CM;
        double perpendicularDistance = PERPENDICULAR_DIR * deltaPerpendicularTicks / TICKS_PER_CM;

        // Correzione dello spostamento causato dalla rotazione
        double localY = parallelDistance - PARALLEL_OFFSET * deltaHeading;
        double localX = perpendicularDistance + PERPENDICULAR_OFFSET * deltaHeading;

        // Dalle coordinate del robot alle coordinate del campo
        double deltaX = localX * Math.cos(averageHeading) - localY * Math.sin(averageHeading);
        double deltaY = localX * Math.sin(averageHeading) + localY * Math.cos(averageHeading);

        x += deltaX;
        y += deltaY;

        direction = Math.toDegrees(currentHeading);
        lastHeading = currentHeading;
    }


    // ===== METODI PUBBLICI =====

    /** Heading in gradi, [-180, 180]. */
    public double getDirection() {
        return direction;
    }

    /** Heading in radianti, [-pi, pi]. */
    public double getHeadingRadians() {
        return lastHeading;
    }

    public double[] getPosition() {
        return new double[]{x, y};
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }


    // Riporta un angolo nell'intervallo [-π, π]
    private double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
