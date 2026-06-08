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

@TeleOp(name="MainTest", group="Main")
public class debug extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();

    boolean DEBUGGING = true;
    double SPEED = .5;
    double PaY = -4.99, PrX = 9.73, R = 2, N = 8192, KP = 2;
    int QR_LIVE_TIME = 1000;
    double CAMERA_OFFSET = 5;
    double cmTickRatio = 2 * Math.PI * R / N;
    double KP_FACTOR = 6.0;



    final double[] pos = {0, 0, 0, 0};
    double hoodPos = .25;
    double shoot = 0;
    double POWER_Q = .22;
    double oParallel = 0, oPerp = 0, oHeading = 0;
    double speed = SPEED;
    double[] lastKnownQR = {-999, -999, 0, 0};
    double TurretPosition = 0.5;



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
        odoParallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoParallel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor lfD = hardwareMap.get(DcMotor.class, "lf");
        DcMotor lbD = hardwareMap.get(DcMotor.class, "lb");
        DcMotor rfD = hardwareMap.get(DcMotor.class, "rf");
        DcMotor rbD = hardwareMap.get(DcMotor.class, "rb");

        lfD.setDirection(DcMotor.Direction.REVERSE);
        lbD.setDirection(DcMotor.Direction.REVERSE);
        rfD.setDirection(DcMotor.Direction.FORWARD);
        rbD.setDirection(DcMotor.Direction.FORWARD);
        lfD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotorEx gianluca = (DcMotorEx) hardwareMap.get(DcMotor.class, "gianluca");
        DcMotorEx Daroui = (DcMotorEx) hardwareMap.get(DcMotor.class, "Daroui");
        DcMotor laZappa = hardwareMap.get(DcMotor.class, "laZappa");
        DcMotor bonolis = hardwareMap.get(DcMotor.class, "bonolis");

        gianluca.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Daroui.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gianluca.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Daroui.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gianluca.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Daroui.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients currentPIDF = gianluca.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients aggressivePIDF = new PIDFCoefficients(currentPIDF.p * KP_FACTOR, currentPIDF.i, currentPIDF.d, currentPIDF.f);
        gianluca.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, aggressivePIDF);
        Daroui.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, aggressivePIDF);

        telemetry.addData("Motors: ", "Ready :)");

        Servo turettaL = hardwareMap.get(Servo.class, "turettaL");
        Servo cecchettinR = hardwareMap.get(Servo.class, "cecchettinR");
        Servo amilcare = hardwareMap.get(Servo.class, "amilcare");
        Servo carlR = hardwareMap.get(Servo.class, "carlR");
        Servo marxL = hardwareMap.get(Servo.class, "marxL");

        telemetry.addData("Servo: ", "ready :)");

        ctx ctx = new ctx(lfD, lbD, rfD, rbD, odoParallel, odoPerp, imu);

        telemetry.addData("ctx: ", "Ready :)");
        telemetry.addData("Status", "Robot Ready :)");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            odometry(ctx);
            double x = pos[0], y = pos[1], h = pos[2];

            if (gamepad1.left_bumper) {
                speed = 1;
            } else if (gamepad1.left_trigger >= 0.1) {
                speed = (SPEED - 0.1) * (1 - gamepad1.left_trigger) + 0.1;
            } else {
                speed = SPEED;
            }

            turettaL.setPosition(TurretPosition);
            cecchettinR.setPosition(TurretPosition);

            double lX = gamepad1.left_stick_x, lY = -gamepad1.left_stick_y;
            lX = Math.abs(lX) < .4 ? 0 : lX;
            lY = Math.abs(lY) < .4 ? 0 : lY;
            double rX = gamepad1.right_stick_x, rY = -gamepad1.right_stick_y;
            rX = Math.abs(rX) < .2 ? 0 : rX;
            rY = Math.abs(rY) < .2 ? 0 : rY;

            double[] MotArr = MotorOut(lX, lY, rX, rY);
            ctx.lFd.setPower(MotArr[0] * speed);
            ctx.lBd.setPower(MotArr[1] * speed);
            ctx.rFd.setPower(MotArr[2] * speed);
            ctx.rBd.setPower(MotArr[3] * speed);

            double output = 0;
            if (!tagProcessor.getDetections().isEmpty() && gamepad2.triangle) {
                List<AprilTagDetection> tags = tagProcessor.getDetections();
                for (AprilTagDetection tag : tags) {
                    if (tag.metadata != null) {
                        lastKnownQR[0] = tag.ftcPose.x;
                        lastKnownQR[1] = tag.ftcPose.y;
                        lastKnownQR[2] = 0;
                        lastKnownQR[3] = tag.ftcPose.yaw;
                    }
                }
            }

            lastKnownQR[2] += timer.milliseconds();
            if (lastKnownQR[2] > QR_LIVE_TIME) lastKnownQR[0] = -999;

            if (lastKnownQR[0] != -999 && gamepad2.triangle) {
                output = (lastKnownQR[1] / 100) / 7 + POWER_Q;
                if (lastKnownQR[1] > 200) {
                    hoodPos = shoot > 700 ? .45 : .56;
                } else {
                    hoodPos = shoot > 700 ? .1 : .2;
                }
                //if (shoot > 700) output += 0.04;
            }

            if (!gamepad2.triangle) {
                output = gamepad2.right_trigger;
            }

            double targetVelocity = output * 2500;
            gianluca.setVelocity(targetVelocity);
            Daroui.setVelocity(targetVelocity);

            double intake = gamepad1.right_trigger > gamepad2.left_trigger ? Math.min(1, gamepad1.right_trigger) : Math.min(1, gamepad2.left_trigger);
            if (!gamepad2.cross) {
                laZappa.setPower(intake);
                bonolis.setPower(intake > 0.1 ? 0.3 : 0);
                amilcare.setPosition(.25);
                shoot = 0;
            } else {
                double currentVel = Math.abs(gianluca.getVelocity());
                boolean flywheelReady = (targetVelocity > 100) && (currentVel >= (targetVelocity - 150));

                if (shoot > 200) {
                    laZappa.setPower(.4);
                    bonolis.setPower(1);
                } else {
                    laZappa.setPower(0);
                    bonolis.setPower(0);
                }
                amilcare.setPosition(0);
                shoot += timer.milliseconds();
            }

            if ((gamepad2.dpad_down && hoodPos > 0) && !gamepad2.triangle) {
                hoodPos -= 0.002 * timer.milliseconds();
            }
            if ((gamepad2.dpad_up && hoodPos < 0.6) && !gamepad2.triangle) {
                hoodPos += 0.002 * timer.milliseconds();
            }

            marxL.setPosition(hoodPos);
            carlR.setPosition(1 - hoodPos);

            if (gamepad2.right_bumper && !gamepad2.triangle) {
                TurretPosition += 0.001 * timer.milliseconds();
                TurretPosition = Math.max(0.0, Math.min(1.0, TurretPosition));
            }
            if (gamepad2.left_bumper && !gamepad2.triangle ) {
                TurretPosition -= 0.001 * timer.milliseconds();
                TurretPosition = Math.max(0.0, Math.min(1.0, TurretPosition));
            }

            telemetry.addData("turettaL",turettaL.getPosition());
            telemetry.addData("cecchettinR",cecchettinR.getPosition());



            telemetry.addData("xyt", "x: %.2f y: %.2f t: %.2f", x, y, Math.toDegrees(h));
            telemetry.addData("Flywheel Target", targetVelocity);
            telemetry.addData("Flywheel Actual", gianluca.getVelocity());
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