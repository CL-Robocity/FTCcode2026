
package org.firstinspires.ftc.teamcode.Odometry;

import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PositionFinder {

    private final GoBildaPinpointDriver pinpoint;

    private double x;
    private double y;
    private double headingRad;

    // Il programma usa:
    // 0° = robot rivolto verso +Y
    // positivo = rotazione antioraria
    //
    // Il Pinpoint usa:
    // 0° = asse +X
    // positivo = rotazione antioraria
    private static final double PINPOINT_HEADING_OFFSET = Math.PI / 2.0;

    public PositionFinder(
            RobotHardware robot,
            double startX,
            double startY,
            double startHeadingDeg
    ) {

        if (robot.pinpoint == null) {
            throw new IllegalStateException(
                    "Pinpoint non inizializzato in RobotHardware!"
            );
        }

        pinpoint = robot.pinpoint;

        // Conversione heading del programma -> heading Pinpoint
        double pinpointHeading = Math.toRadians(startHeadingDeg)
                + PINPOINT_HEADING_OFFSET;

        // Imposta la posizione iniziale del robot nel campo
        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.CM,
                        startX,
                        startY,
                        AngleUnit.RADIANS,
                        pinpointHeading
                )
        );

        update();
    }

    // Aggiornamento della posizione
    public void update() {

        pinpoint.update();

        // Posizione in centimetri
        x = pinpoint.getPosX(DistanceUnit.CM);
        y = pinpoint.getPosY(DistanceUnit.CM);

        // Heading Pinpoint -> heading del programma
        double pinpointHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        headingRad = normalizeRadians(
                pinpointHeading - PINPOINT_HEADING_OFFSET
        );
    }

    // Restituisce l'heading in gradi [0, 360)
    public double getDirection() {
        return normalizeDegrees(Math.toDegrees(headingRad));
    }

    // Restituisce l'heading in radianti
    public double getHeadingRadians() {
        return headingRad;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    private double normalizeDegrees(double angle) {
        angle %= 360.0;

        if (angle < 0) {
            angle += 360.0;
        }

        return angle;
    }
}
