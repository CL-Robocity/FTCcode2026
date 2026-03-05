package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name="TurretHoming", group="Main")
public class turretHoming extends LinearOpMode {

    //MAIN GLOBAL CONSTANTS
    int TURRET_OFFSET = 1320; //Turret Starting Position

    @Override
    public void runOpMode() throws InterruptedException {
        //Dashboard Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Wait");
        telemetry.update();

        //DcMotors and Servos Init
        DcMotor turetta = hardwareMap.get(DcMotor.class, "turetta"); //Torretta

        //Sensors
        TouchSensor toccami = hardwareMap.get(TouchSensor.class, "toccami"); //Homing Touch Sensor

        //Turret Homing Process
        telemetry.addData("Status", "Homing Turret");
        telemetry.update();
        waitForStart();

        turretHoming(toccami, turetta);
    }

    //Turret Starting Alignment ( homing )
    private void turretHoming(TouchSensor toccami, DcMotor turetta) {

        //Move until u know where u are
        while (!isStopRequested() && !toccami.isPressed()) {
            turetta.setPower(-0.2);
            idle();
        }

        //STOP e dai la precedenza
        turetta.setPower(0);

        //Reset encoder
        turetta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turetta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Robot Hardware context class
    static class ctx {
        public final DcMotor lFd; //Left Front Drive
        public final DcMotor lBd; //Left Back Drive
        public final DcMotor rFd; //Right Front Drive
        public final DcMotor rBd; //Right back Drive
        public final DcMotor odoParallel; //Parallel Odometry
        public final DcMotor odoPerp; //Perpendicular Odometry
        public final IMU imu; //Inertial Mesurement Unit

        public ctx(DcMotor lFd, DcMotor lBd, DcMotor rFd, DcMotor rBd, DcMotor odoParallel, DcMotor odoPerp, IMU imu) {
            this.lFd = lFd;
            this.lBd = lBd;
            this.rFd = rFd;
            this.rBd = rBd;
            this.odoParallel = odoParallel;
            this.odoPerp = odoPerp;
            this.imu = imu;
        }
    }
}