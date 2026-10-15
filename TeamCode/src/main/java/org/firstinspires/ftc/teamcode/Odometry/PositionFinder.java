
package org.firstinspires.ftc.teamcode.Odometry;

import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PositionFinder {

    // ===== PARAMETRI DA CALIBRARE =====

    // Tick dell'encoder necessari per percorrere 1 cm
    private static final double TICKS_PER_CM = 100.0;

    // Posizione della ruota parallela rispetto al centro:
    // positiva se si trova a destra del centro del robot
    private static final double PARALLEL_OFFSET = 0.0;

    // Posizione della ruota perpendicolare rispetto al centro:
    // positiva se si trova verso la parte anteriore del robot
    private static final double PERPENDICULAR_OFFSET = 0.0;


    // ===== HARDWARE E POSIZIONE =====

    private final RobotHardware Robot;

    private double x;
    private double y;
    private double direction;

    private int lastParallelTicks;
    private int lastPerpendicularTicks;

    private double lastHeading;


    // ===== COSTRUTTORE =====

    public PositionFinder(
            RobotHardware robot,
            double initialX,
            double initialY
    ) {
        this.Robot = robot;

        // Coordinate iniziali assolute rispetto all'angolo (0,0)
        this.x = initialX;
        this.y = initialY;

        // Memorizza le letture iniziali degli encoder
        lastParallelTicks = Robot.odoParallel.getCurrentPosition();
        lastPerpendicularTicks = Robot.odoPerp.getCurrentPosition();

        // La direzione iniziale viene considerata pari a zero
        lastHeading = getRawHeading();
        direction = 0.0;
    }


    // ===== LETTURA IMU =====

    private double getRawHeading() {
        return Robot.imu
                .getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
    }

    private double getDirectionRadians() {
        return normalizeAngle(getRawHeading() - initialHeading);
    }

    private double initialHeading = 0.0;


    // ===== AGGIORNAMENTO POSIZIONE =====

    public void update() {

        // Lettura encoder attuali
        int currentParallelTicks =
                Robot.odoParallel.getCurrentPosition();

        int currentPerpendicularTicks =
                Robot.odoPerp.getCurrentPosition();

        // Differenza rispetto al ciclo precedente
        int deltaParallelTicks =
                currentParallelTicks - lastParallelTicks;

        int deltaPerpendicularTicks =
                currentPerpendicularTicks - lastPerpendicularTicks;

        // Aggiorna i riferimenti per il prossimo ciclo
        lastParallelTicks = currentParallelTicks;
        lastPerpendicularTicks = currentPerpendicularTicks;


        // Direzione attuale e variazione angolare
        double currentHeading = getDirectionRadians();
        currentHeading = normalizeAngle(currentHeading);

        double deltaHeading =
                normalizeAngle(currentHeading - lastHeading);

        // Angolo medio durante lo spostamento
        double averageHeading =
                lastHeading + deltaHeading / 2.0;


        // Conversione da tick a centimetri
        double parallelDistance =
                deltaParallelTicks / TICKS_PER_CM;

        double perpendicularDistance =
                deltaPerpendicularTicks / TICKS_PER_CM;


        // Correzione dello spostamento causato dalla rotazione
        double localY =
                parallelDistance - PARALLEL_OFFSET * deltaHeading;

        double localX =
                perpendicularDistance
                        + PERPENDICULAR_OFFSET * deltaHeading;


        // Trasformazione dalle coordinate del robot
        // alle coordinate del campo.
        double deltaX =
                localX * Math.cos(averageHeading)
                        - localY * Math.sin(averageHeading);

        double deltaY =
                localX * Math.sin(averageHeading)
                        + localY * Math.cos(averageHeading);


        // Aggiornamento coordinate assolute
        x += deltaX;
        y += deltaY;

        direction = Math.toDegrees(currentHeading);
        lastHeading = currentHeading;
    }


    // ===== METODI PUBBLICI =====

    public double getDirection() {
        return direction;
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
