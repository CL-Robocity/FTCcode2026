
package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.Movmts.StandardMvmnt;
import org.firstinspires.ftc.teamcode.Odometry.PositionFinder;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "odometryTest", group = "TEST")
public class odometryTest extends LinearOpMode {

    private final RobotHardware Robot = new RobotHardware();
    private StandardMvmnt Mvmnt;
    private PositionFinder position;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Inizializzazione hardware
        Robot.init(hardwareMap, telemetry);

        // Inizializzazione movimento
        Mvmnt = new StandardMvmnt(Robot, gamepad1);

        // Coordinate iniziali del centro del robot, in centimetri
        position = new PositionFinder(Robot, 120.0, 60.0);

        telemetry.addLine("Odometry inizializzata");
        telemetry.addData("X iniziale (cm)", position.getX());
        telemetry.addData("Y iniziale (cm)", position.getY());
        telemetry.update();

        waitForStart();

        timer.reset();

        while (opModeIsActive()) {

            // Aggiorna posizione e direzione
            position.update();

            // Controlla il movimento tramite gamepad
            Mvmnt.move();

            // Legge le coordinate aggiornate
            double x = position.getX();
            double y = position.getY();
            double direction = position.getDirection();

            // Visualizza i dati
            telemetry.addData("X (cm)", "%.2f", x);
            telemetry.addData("Y (cm)", "%.2f", y);
            telemetry.addData("Direzione (gradi)", "%.2f", direction);
            telemetry.addData("Tempo (s)", "%.2f", timer.seconds());

            telemetry.update();
        }
    }
}
