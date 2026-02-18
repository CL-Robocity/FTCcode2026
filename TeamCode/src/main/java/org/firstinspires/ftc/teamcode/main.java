package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DigitalChannel;

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
    int TURRET_MAX = 2400;
    int TURRET_MIN = -20;
    int WRAPPING_TICKS = 2;
    int QR_LIVE_TIME = 500;

    double cmTickRatio = 2 * Math.PI * R / N;
    final double[] pos = {0, 0, 0};

    int turettaTarget = 90;

    boolean hLock = false;
    double targetH = 0, hoodPos = .5;

    @Override
    public void runOpMode() throws InterruptedException {
        //--- Dashboard Init ---
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "ASPETTA UN ATTIMO");
        telemetry.update();
        //---

        //--- Camera Init ---
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTag(20, "Blu", 41, DistanceUnit.CM)
                .addTag(24, "Red", 41, DistanceUnit.CM)
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

        setManualExposure(visionPortal, 1, 200);
        FtcDashboard.getInstance().startCameraStream(visionPortal, 24);
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
        DcMotor odoY = hardwareMap.get(DcMotor.class, "odo_y");

        odoY.setDirection(DcMotor.Direction.REVERSE);

        odoParallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odoParallel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        DcMotor gianluca = hardwareMap.get(DcMotor.class, "gianluca");
        DcMotor in = hardwareMap.get(DcMotor.class, "in");
        DcMotor turetta = hardwareMap.get(DcMotor.class, "turetta");

        in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo levetta = hardwareMap.get(Servo.class, "levetta");
        Servo outL = hardwareMap.get(Servo.class, "outL");
        Servo outR = hardwareMap.get(Servo.class, "outR");

        /*
        dcMotor.setDirection(DcMotor.Direction.FORWARD);
        dcMotor.setTargetPosition(0);
        dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */
        //---

        //Sensors

        NormalizedColorSensor colore = hardwareMap.get(NormalizedColorSensor.class, "colors");
        TouchSensor toccami = hardwareMap.get(TouchSensor.class, "toccami");

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
        long levettaTime = 0, levettaWaiter = 0;
        boolean levettaBool = false;
        double input;
        double[] lastKnownQR = {-999, -999, 0};

        int isWrapping = 0;
        int wrapTarget = 0;

        turretHoming(toccami, turetta);

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

            double[] MotArr = MotorOut(lX, lY, rX, rY);

            telemetry.addData("DriveMotors", "%.2f %.2f %.2f %.2f", MotArr[0], MotArr[1], MotArr[2], MotArr[3]);

            ctx.lFd.setPower(MotArr[0] * speed);
            ctx.lBd.setPower(MotArr[1] * speed);
            ctx.rFd.setPower(MotArr[2] * speed);
            ctx.rBd.setPower(MotArr[3] * speed);
            //---

            //--- QR Code ---
            input = 0;
            if (!tagProcessor.getDetections().isEmpty()) {
                List<AprilTagDetection> tags = tagProcessor.getDetections();
                for (AprilTagDetection tag : tags) {
                    if (tag.metadata != null) {
                        double tx = tag.ftcPose.x, ty = tag.ftcPose.y;

                        if (gamepad2.triangle) {
                            lastKnownQR[0] = tx;
                            lastKnownQR[1] = ty;
                            lastKnownQR[2] = 0;
                        }
                    }
                }
            } else {
                telemetry.addLine("No AprilTags detected");
            }

            lastKnownQR[2]+=timer.milliseconds();
            if (lastKnownQR[2] > QR_LIVE_TIME) lastKnownQR[0] = -999;
            if (Math.abs(lastKnownQR[0]) > 10 && lastKnownQR[0]!= -999) {
                double trackSpeed = lastKnownQR[2] < 3 ? Math.pow(lastKnownQR[0], 2)/lastKnownQR[1] * 0.01 : 0.01;
                telemetry.addData("TEST", trackSpeed);

                if (Math.signum(lastKnownQR[0]) == 1) {input = -Math.min(0.2, Math.max(trackSpeed, 0.01));}
                else {input = Math.min(0.2, Math.max(trackSpeed, 0.01));}
            } else {
                input = 0;
            }

            if (Math.abs(rX) > 0.05 && gamepad2.triangle) {
                input = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate * -0.19285;
            }
            //---

            //--- CODE --

            //IL BRO CHE SPARA

            if (!gamepad2.triangle) gianluca.setPower(gamepad2.right_trigger);
            if (!levettaBool) in.setPower(gamepad2.left_trigger);

            if (!gamepad2.cross) {
                levettaWaiter = System.currentTimeMillis();
            }

            if (gamepad2.cross && !levettaBool) {
                in.setPower(1);
                levettaTime = System.currentTimeMillis();
                levettaBool = true;
            }

            if (levettaBool) {
                in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                long dt = System.currentTimeMillis() - levettaTime;
                NormalizedRGBA rgb = colore.getNormalizedColors();

                if (dt < 300) {
                    if (rgb.blue > 0.001) {levetta.setPosition(.75);}
                } else if (dt < 900) {
                    levetta.setPosition(0.43);
                } else {
                    levetta.setPosition(0.43);
                    levettaBool = false;
                }

                if (System.currentTimeMillis() - levettaWaiter > 800 && dt > 300) {
                    in.setPower(1);
                } else {
                    in.setPower(0);
                }

            } else {
                levetta.setPosition(0.44);
                in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (gamepad2.dpad_down && hoodPos > 0.47) {
                hoodPos-=0.01;
            }
            if (gamepad2.dpad_up && hoodPos < 0.8) {
                hoodPos+=0.01;
            }

            outL.setPosition(hoodPos);
            outR.setPosition(1-hoodPos);

            if (!gamepad2.triangle) {
                if (gamepad2.left_bumper) input = 0.2;
                else if (gamepad2.right_bumper) input = -0.2;
            }

            telemetry.addData("input", input);

            int pos = turetta.getCurrentPosition();

            if (isWrapping == 0 && pos >= TURRET_MAX && input > 0) {
                isWrapping = -1;
                wrapTarget = TURRET_MIN;
                turetta.setTargetPosition(wrapTarget);
                turetta.setPower(0);
            }

            else if (isWrapping == 0 && pos <= TURRET_MIN && input < 0) {
                isWrapping = 1;
                wrapTarget = TURRET_MAX;
                turetta.setPower(0);
            }

            if (isWrapping != 0) {

                //if (Math.abs(rX) > 0.05 && gamepad2.triangle) {input = Math.pow(input, 2) * Math.signum(input) * 20;}
                if ((isWrapping == -1 && input > 0) || (isWrapping == 1 && input < 0)) {
                    wrapTarget+= (int) (input*WRAPPING_TICKS*timer.milliseconds());
                    turetta.setTargetPosition(wrapTarget);
                }
                turetta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                int d = Math.min(Math.abs(wrapTarget - pos), Math.abs(isWrapping == 1 ? pos : TURRET_MAX-pos));

                telemetry.addData("d", wrapTarget);

                double wrapPower = d * 0.003;
                wrapPower = Math.max(0.2, wrapPower);
                wrapPower = Math.min(1, wrapPower);
                wrapPower *= isWrapping == 1 ? 1 : -1;

                if ((isWrapping == 1 && pos > wrapTarget) || (isWrapping == -1 && pos < wrapTarget)) {
                    isWrapping = 0;

                    turetta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    turetta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                turetta.setPower(wrapPower);
            } else {
                if (gamepad2.triangle) turetta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turetta.setPower(input);
            }

            //---

            if (gamepad1.left_bumper) {
                speed  = 1;
            } else if (gamepad1.left_trigger >= 0.1){
                speed  = (SPEED - 0.1)*(1 - gamepad1.left_trigger) + 0.1;
            } else {
                speed  = SPEED;
            }

            //telemetry.addData("lra", "l: %6d r: %6d a: %6d", oParallel, oPerp, oHeading);
            //telemetry.addData("xyt", "x: %.2f y: %.2f t: %.2f", x, y, Math.toDegrees(h));
            telemetry.addData("loop", "%.1f ms", timer.milliseconds());

            telemetry.update();
            timer.reset();
        }

        if (odometryThread.isAlive()) {
            odometryThread.interrupt();
            odometryThread.join();
        }

        visionPortal.close();
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
            //rot = e * KP;
            rot = 0;
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
        while (angle < -Math.PI) angle += 2*Math.PI;
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

    private void turretHoming(TouchSensor toccami, DcMotor turetta) {
        while (!toccami.isPressed()) {
            turetta.setPower(-0.2);
        }
        turetta.setPower(0);
        turetta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turetta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turetta.setPower(0.1);
        turettaTarget = 1300;

        while (turetta.getCurrentPosition() < turettaTarget) {}
        turetta.setPower(0);

        return;
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