
package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.Movmts.StandardMvmnt;
import org.firstinspires.ftc.teamcode.Odometry.PositionFinder;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MotorTest", group = "TEST")
public class motorTest extends LinearOpMode {

    private final RobotHardware Robot = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {

        // Inizializzazione hardware
        Robot.init(hardwareMap, telemetry);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.left_bumper){
                Robot.TopFlyWheel.setVelocity(6000);
                Robot.DownFlyWheel.setVelocity(6000);
            }else{
                Robot.TopFlyWheel.setVelocity(0);
                Robot.DownFlyWheel.setVelocity(0);
            }
            telemetry.addData("MotorVelocity --> ", Robot.TopFlyWheel.getVelocity());
        }
    }
}
