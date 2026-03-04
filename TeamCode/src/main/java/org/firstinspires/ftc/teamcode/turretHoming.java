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

    //timer obj
    ElapsedTime timer = new ElapsedTime();

    //MAIN GLOBAL CONSTANTS
    boolean DEBUGGING = false; //Debugging Const
    double SPEED = .5; //Robot Speed
    double PaY = -4.99, PrX = 9.73, R = 2, N = 8192, KP = 2; //Odometry Constants
    int TURRET_OFFSET = 1320; //Turret Starting Position
    int TURRET_MAX = 2600, TURRET_MIN = -70; //Turret Constraints
    double AUTOAIM_MIN_SPEED = 0.01, AUTOAIM_MAX_SPEED = 0.2; //Auto-Aiming Speed
    int QR_LIVE_TIME = 1000; //QR Code Expire time
    double CAMERA_OFFSET = 5; //Camera Offset
    double RAD_TO_TICKS = 1325/Math.PI; //Turret Angle to Motor Ticks
    double POWER_TO_TICKS = 3.5; //Motor Power to Turret Ticks
    double TURRET_ACCEL = 0.001; //Turret Acceleration
    double ERR = 10;
    double cmTickRatio = 2 * Math.PI * R / N;

    //MAIN GLOBAL VARIABLES
    final double[] pos = {0, 0, 0, 0}; //Global Robot x, y, h, Δh
    double hoodPos = .5; //Hood Position
    double tRawPos = TURRET_OFFSET;
    double oParallel = 0, oPerp = 0, oHeading = 0, oTurret = TURRET_OFFSET; //Old Odometry values vars, Old Turret Pos
    double[] turretLock = {-999, 0}; //Turret Lock Position
    double speed = SPEED; //Robot Current Speed
    long levettaTime = 0, levettaWaiter = 0; //Outtake server clock
    int levettaBool = 0; //Levetta cycle activator
    double[] lastKnownQR = {-999, -999, 0, 0}; //Last QRcode saved
    double hoodError = 0, outputError = 1;

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

        //Move to offset position
        turetta.setPower(0.1);
        while (!isStopRequested() && turetta.getCurrentPosition() < TURRET_OFFSET) {
            idle();
        }
        turetta.setPower(0);
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