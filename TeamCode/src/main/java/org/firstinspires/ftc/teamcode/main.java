package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="MainDrive", group="Main")
public class main extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();

    double oParallel, oPerp, oHeading; //current position, old position -> Left, Right, Aux

    //173,5 180,5
    double SPEED = .5;
    double PaY = 1, PrX = 1, R = 2, N = 8192, KP = 2;

    double cmTickRatio = 2 * Math.PI * R / N;
    final double[] pos = {0, 0, 0};

    boolean hLock = false;
    double targetH = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //--- Dashboard Init ---
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "ASPETTA UN ATTIMO");
        telemetry.update();
        //---

        //--- Camera Init ---
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTag(21, "jesus", 41, DistanceUnit.CM)
                .build();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(629.694, 629.694, 358.384, 256.314)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();

        setManualExposure(visionPortal, 2, 250);
        FtcDashboard.getInstance().startCameraStream(visionPortal, 60);
        //---

        //--- IMU ---
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();
        //---

        //--- Odometry Encoders Init ---
        DcMotor odoParallel = hardwareMap.get(DcMotor.class, "lb");
        DcMotor odoPerp = hardwareMap.get(DcMotor.class, "rb");

        odoParallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odoParallel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //---

        //--- DriveMotors Init ---
        DcMotor lfD = hardwareMap.get(DcMotor.class, "lf");
        DcMotor lbD = hardwareMap.get(DcMotor.class, "lb");
        DcMotor rfD = hardwareMap.get(DcMotor.class, "rf");
        DcMotor rbD = hardwareMap.get(DcMotor.class, "rb");

        lfD.setDirection(DcMotor.Direction.REVERSE);
        lbD.setDirection(DcMotor.Direction.REVERSE);
        rbD.setDirection(DcMotor.Direction.FORWARD);
        rfD.setDirection(DcMotor.Direction.FORWARD);

        lfD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lfD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //---

        //--- Motors and Servos Init ---

        /*DcMotor testMotor = hardwareMap.get(DcMotor.class, "luigi");
        Servo testServo = hardwareMap.get(Servo.class, "gobildatest");*/

        /*
        dcMotor.setDirection(DcMotor.Direction.FORWARD);
        dcMotor.setTargetPosition(0);
        dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */
        //---

        //Robot Context Init
        ctx ctx = new ctx(lfD, lbD, rfD, rbD, odoParallel, odoPerp, imu);

        //--- Odo Thread Init

        Thread odometryThread = new Thread(() -> {
            final long periodNs = 5_000_000; // 200 Hz
            Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
            long nextTime = System.nanoTime() + periodNs;

            while (!isStopRequested()) {
                odometry(ctx);

                long sleepTime = nextTime - System.nanoTime();

                if (sleepTime > 0) {
                    try {
                    TimeUnit.NANOSECONDS.sleep(sleepTime);
                    } catch (InterruptedException e) {
                        break;
                    }
                } else {
                    nextTime = System.nanoTime();
                }
            }
        });

        //---
        double speed = SPEED;

        telemetry.addData("Status", "VAII");
        telemetry.update();

        waitForStart();

        odometryThread.start();
        timer.reset();

        while (opModeIsActive()) {
            double x, y, h;
            synchronized(pos) {
                x = pos[0];
                y = pos[1];
                h = pos[2];
            }

            //--- Mecanum Drive
            double lX = gamepad1.left_stick_x, lY = -gamepad1.left_stick_y;
            lX = Math.abs(lX) < .4 ? 0 : lX; //TODO -> Tune Dead Zone :)
            lY = Math.abs(lY) < .4 ? 0 : lY;
            double rX = gamepad1.right_stick_x, rY = -gamepad1.right_stick_y;
            rX = Math.abs(rX) < .2 ? 0 : rX;
            rY = Math.abs(rY) < .2 ? 0 : rY;

            double[] MotArr = MotorOut(-lX, -lY, rX, rY);

            telemetry.addData("DriveMotors", "%.2f %.2f %.2f %.2f", MotArr[0], MotArr[1], MotArr[2], MotArr[3]);

            ctx.lFd.setPower(MotArr[0] * speed);
            ctx.lBd.setPower(MotArr[1] * speed);
            ctx.rFd.setPower(MotArr[2] * speed);
            ctx.rBd.setPower(MotArr[3] * speed);
            //---

            //--- QR Code ---
            if (!tagProcessor.getDetections().isEmpty()) {
                List<AprilTagDetection> tags = tagProcessor.getDetections();
                for (AprilTagDetection tag : tags) {
                    if (tag.metadata != null) {
                        double tx = tag.ftcPose.x, ty = tag.ftcPose.y;
                        double ta = Math.atan2(tx, ty)*360/(2*Math.PI);

                        telemetry.addData("QRcode", ta);
                    }
                }
            } else {
                telemetry.addLine("No AprilTags detected");
            }
            //---

            //--- CODE ---



            //---

            if (gamepad1.left_bumper) {
                speed  = 1;
            } else if (gamepad1.left_trigger >= 0.1){
                speed  = (SPEED - 0.1)*(1 - gamepad1.left_trigger) + 0.1;
            } else {
                speed  = SPEED;
            }

            telemetry.addData("lra", "l: %6d r: %6d a: %6d", oParallel, oPerp, oHeading);
            telemetry.addData("xyt", "x: %.2f y: %.2f t: %.2f", x, y, Math.toDegrees(h));
            telemetry.addData("loop", "%.1f ms", timer.milliseconds());
            telemetry.update();
            timer.reset();
        }

        if (odometryThread.isAlive()) {
            odometryThread.interrupt();
            odometryThread.join();
        }
    }

    private double[] MotorOut(double lX, double lY, double rX, double rY) {
        double h = pos[2];
        double rot;
        if (Math.abs(rX) < .05) {

            if (!hLock) {
                targetH = h;
                hLock = true;
            }

            double e = angleWrap(targetH - h);
            rot = e * KP;
        } else {
            hLock = false;
            rot = rX;
            targetH = h;
        }

        double lf = lY + lX + rot;
        double lb = lY - lX + rot;
        double rf = lY - lX - rot;
        double rb = lY + lX - rot;

        double max = Math.max(1, Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb)))));
        return new double[]{lf/max, lb/max, rf/max, rb/max}; //lf, lb, rf, rb
    }

    private double getAng(double a, int dir) {
        return 0.5 + 1f/150f*a*dir; //1dx -1sx
    }

    private void odometry(ctx ctx) {
        double parallel = ctx.odoParallel.getCurrentPosition() * cmTickRatio;
        double perp = ctx.odoPerp.getCurrentPosition() * cmTickRatio;

        double imuHeading = getHeading(ctx);

        double dParallel = parallel - oParallel;
        double dPerp = perp - oPerp;
        double dHeading = angleWrap(imuHeading - oHeading);

        oParallel = parallel;
        oPerp = perp;
        oHeading = imuHeading;

        double corrX = dParallel - dHeading * PaY;
        double corrY = dPerp + dHeading * PrX;
        double midHeading = pos[2] + dHeading/2;

        double cos = Math.cos(midHeading), sin = Math.sin(midHeading);

        synchronized(pos) {
            pos[0] += corrX * cos - corrY * sin;
            pos[1] += corrX * sin + corrY * cos;
            pos[2] += dHeading;
            pos[2] = angleWrap(pos[2]);
        }
    }

    private double getHeading(ctx ctx) {
        return ctx.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2*Math.PI;
        while (angle < Math.PI) angle += 2*Math.PI;
        return angle;
    }

    private void setManualExposure(VisionPortal visionPortal, int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    static class ctx {
        public final DcMotor lFd;
        public final DcMotor lBd;
        public final DcMotor rFd;
        public final DcMotor rBd;
        public final DcMotor odoParallel;
        public final DcMotor odoPerp;
        public final IMU imu;

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