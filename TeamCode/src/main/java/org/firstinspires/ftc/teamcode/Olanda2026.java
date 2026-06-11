package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.List;

@TeleOp(name="RedbullGivesYouWiiiings", group="Main")
public class Olanda2026 extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();

    // costanti di sistema
    final boolean DEBUGGING = true;
    final double SPEED = .5;
    final double PaY = -4.99, PrX = 9.73, R = 2, N = 8192, KP = 2;
    final int QR_LIVE_TIME = 1000;
    final double CAMERA_OFFSET = 5;
    final double cmTickRatio = 2 * Math.PI * R / N;
    final double KP_FACTOR = 6.0;



    final double[] pos = {0, 0, 0, 0};
    double hoodPos = .25;
    double shootTime = 0;
    double POWER_Q = .28;
    double oParallel = 0, oPerp = 0, oHeading = 0;
    double speed = SPEED;
    double[] lastKnownQR = {-999, -999, 0, 0};
    double TurretPosition = 0.5;





    // variabili in game

    double lX, lY, rX, rY;
    double[] MotorArr;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        );
        imu.initialize(parameters);
        sleep(1000);
        imu.resetYaw();


        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTag(20, "Blu", 41, DistanceUnit.CM)
                .addTag(24, "Red", 41, DistanceUnit.CM)
                .addTag(23, "Giacomo", 41, DistanceUnit.CM)
                .addTag(21, "jesus", 41, DistanceUnit.CM)
                .build();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setDrawAxes(DEBUGGING)
                .setDrawTagOutline(DEBUGGING)
                .setDrawTagID(DEBUGGING)
                .setDrawCubeProjection(DEBUGGING)
                .setLensIntrinsics(629.694, 629.694, 358.384, 256.314)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(DEBUGGING)
                .build();

        telemetry.addData("Vision Portal: ", "Ready :)");

        if (DEBUGGING) FtcDashboard.getInstance().startCameraStream(visionPortal, 24);

        DcMotor odoParallel = hardwareMap.get(DcMotor.class, "bonolis");
        DcMotor odoPerp = hardwareMap.get(DcMotor.class, "laZappa");

        // reset encoder delle odo
        odoParallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoParallel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // riconoscimento motori di movimento
        DcMotor lfD = hardwareMap.get(DcMotor.class, "lf");
        DcMotor lbD = hardwareMap.get(DcMotor.class, "lb");
        DcMotor rfD = hardwareMap.get(DcMotor.class, "rf");
        DcMotor rbD = hardwareMap.get(DcMotor.class, "rb");

        // set direzione motori
        lfD.setDirection(DcMotor.Direction.REVERSE);
        lbD.setDirection(DcMotor.Direction.REVERSE);
        rfD.setDirection(DcMotor.Direction.FORWARD);
        rbD.setDirection(DcMotor.Direction.FORWARD);

        // set per runnare i motori
        lfD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set movimento a potenza 0
        lfD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotorEx TopFlyWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "gianluca");
        DcMotorEx DownFlyWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Daroui");
        DcMotor FrontIntake = hardwareMap.get(DcMotor.class, "laZappa");
        DcMotor IntakeRoller = hardwareMap.get(DcMotor.class, "bonolis");

        TopFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DownFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TopFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DownFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        TopFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DownFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients currentPIDF = TopFlyWheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients aggressivePIDF = new PIDFCoefficients(currentPIDF.p * KP_FACTOR, currentPIDF.i, currentPIDF.d, currentPIDF.f);
        TopFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, aggressivePIDF);
        DownFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, aggressivePIDF);

        telemetry.addData("Motors: ", "Ready :)");

        Servo LeftTurretServo = hardwareMap.get(Servo.class, "turettaL");
        Servo RightTurretServo = hardwareMap.get(Servo.class, "cecchettinR");
        Servo BallStopServo = hardwareMap.get(Servo.class, "amilcare");
        Servo LeftHood_Servo = hardwareMap.get(Servo.class, "marxL");
        Servo RightHood_Servo = hardwareMap.get(Servo.class, "carlR");

        telemetry.addData("Servo: ", "Ready :)");

        ctx ctx = new ctx(lfD, lbD, rfD, rbD, odoParallel, odoPerp, imu);


        telemetry.addData("ctx: ", "Ready :)");
        telemetry.addData("\nStatus", "Robot Ready :)");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            odometry(ctx);

            LeftTurretServo.setPosition(TurretPosition);
            RightTurretServo.setPosition(TurretPosition);

            RightHood_Servo.setPosition(TurretPosition);
            LeftHood_Servo.setPosition(TurretPosition);

            // set joystick sinistro
            lX = gamepad1.left_stick_x;
            lY = -gamepad1.left_stick_y;

            // calcolo errore
            lX = Math.abs(lX) < .4 ? 0 : lX;
            lY = Math.abs(lY) < .4 ? 0 : lY;

            // set joystick destro
            rX = gamepad1.right_stick_x;
            rY = -gamepad1.right_stick_y;

            //calcolo errore
            rX = Math.abs(rX) < .2 ? 0 : rX;
            rY = Math.abs(rY) < .2 ? 0 : rY;


            //definizione array motori e output movimento motori
            MotorArr = MotorOut(lX, lY, rX, rY);
            ctx.lFd.setPower(MotorArr[0] * speed);
            ctx.lBd.setPower(MotorArr[1] * speed);
            ctx.rFd.setPower(MotorArr[2] * speed);
            ctx.rBd.setPower(MotorArr[3] * speed);






            telemetry.addData("LeftTurretServo",LeftTurretServo.getPosition());
            telemetry.addData("RigheTurretServo",RightTurretServo.getPosition());

            telemetry.addData("HoodLeft", LeftHood_Servo.getPosition());
            telemetry.addData("HoodRight", RightHood_Servo.getPosition());



            //telemetry.addData("xyt", "x: %.2f y: %.2f t: %.2f", x, y, Math.toDegrees(h));
            //telemetry.addData("Flywheel Target", targetVelocity);
            telemetry.addData("Flywheel Actual", TopFlyWheel.getVelocity());

            telemetry.addData("Odo Parallel",odoParallel.getCurrentPosition());
            telemetry.addData("Odo Perpendicular",odoPerp.getCurrentPosition());
            telemetry.update();

            idle();
            sleep(5);
            timer.reset();
        }
        visionPortal.close();
    }



















    private double[] MotorOut(double lX, double lY, double rX, double rY) {
        double rot = rX;
        double y = lY, x = lX;
        double lf = y + x + rot;
        double lb = y - x + rot;
        double rf = y - x - rot;
        double rb = y + x - rot;
        double max = Math.max(1, Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb)))));
        return new double[]{lf / max, lb / max, rf / max, rb / max};
    }

    private void odometry(ctx ctx) {
        double parallel = ctx.odoParallel.getCurrentPosition() * cmTickRatio;
        double perp = ctx.odoPerp.getCurrentPosition() * cmTickRatio;
        double imuHeading;
        try {
            imuHeading = ctx.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        } catch (Exception e) {
            imuHeading = oHeading;
        }
        double dParallel = parallel - oParallel;
        double dPerp = perp - oPerp;
        double dHeading = angleWrap(imuHeading - oHeading);

        oParallel = parallel;
        oPerp = perp;
        oHeading = imuHeading;

        double corrX = dParallel - dHeading * PaY;
        double corrY = dPerp + dHeading * PrX;
        double midHeading = oHeading + dHeading / 2;
        double cos = Math.cos(midHeading), sin = Math.sin(midHeading);

        pos[0] += corrX * cos - corrY * sin;
        pos[1] += corrX * sin + corrY * cos;
        pos[2] = angleWrap(imuHeading);
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    static class ctx {
        public final DcMotor lFd, lBd, rFd, rBd, odoParallel, odoPerp;
        public final IMU imu;
        public ctx(DcMotor lFd, DcMotor lBd, DcMotor rFd, DcMotor rBd, DcMotor odoParallel, DcMotor odoPerp, IMU imu) {
            this.lFd = lFd; this.lBd = lBd; this.rFd = rFd; this.rBd = rBd;
            this.odoParallel = odoParallel; this.odoPerp = odoPerp; this.imu = imu;
        }
    }
}