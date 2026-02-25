package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

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

    //timer obj
    ElapsedTime timer = new ElapsedTime();

    //MAIN GLOBAL CONSTANTS
    double SPEED = .5; //Robot Speed
    double PaY = 6.8, PrX = 15.4, R = 2, N = 8192, KP = 2; //Odometry Constants
    int TURRET_OFFSET = 1320;
    int TURRET_MAX = 2400, TURRET_MIN = -20; //Turret Constraints
    double AUTOAIM_MIN_SPEED = 0.01, AUTOAIM_MAX_SPEED = 0.2; //Auto-Aiming Speed
    double ANGULAR_TO_RAW = 0.19285;
    int WRAPPING_TICKS = 2; //Turret Wrapping Ticks
    int QR_LIVE_TIME = 500; //QR Code Expire time
    double cmTickRatio = 2 * Math.PI * R / N;

    //MAIN GLOBAL VARIABLES
    final double[] pos = {0, 0, 0}; //Global Robot x, y, h
    int turettaTarget = 90; //Homing temp var
    boolean hLock = false;
    double targetH = 0; //Target Heading
    double hoodPos = .5; //Hood Position
    double oParallel = 0, oPerp = 0, oHeading = 0; //Old Odometry values vars

    @Override
    public void runOpMode() throws InterruptedException {
        //Dashboard Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "ASPETTA UN ATTIMO");
        telemetry.update();
        //---

        //Camera Init
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

        //IMU Init
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();

        //Odometry Encoders Init
        DcMotor odoParallel = hardwareMap.get(DcMotor.class, "in"); //Parallel Encoder
        DcMotor odoPerp = hardwareMap.get(DcMotor.class, "odo_y"); //Perpendicular Encoder

        odoParallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odoParallel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //riveMotors Init
        DcMotor lfD = hardwareMap.get(DcMotor.class, "lf"); //Left Front
        DcMotor lbD = hardwareMap.get(DcMotor.class, "lb"); //Left Back
        DcMotor rfD = hardwareMap.get(DcMotor.class, "rf"); //Right Front
        DcMotor rbD = hardwareMap.get(DcMotor.class, "rb"); //Right Back

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

        //DcMotors and Servos Init
        DcMotor gianluca = hardwareMap.get(DcMotor.class, "gianluca"); //Flywheel
        DcMotor in = hardwareMap.get(DcMotor.class, "in"); //Intake
        DcMotor turetta = hardwareMap.get(DcMotor.class, "turetta"); //Torretta

        in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo levetta = hardwareMap.get(Servo.class, "levetta"); //Outtake server
        Servo outL = hardwareMap.get(Servo.class, "outL"); //Outtake hood left
        Servo outR = hardwareMap.get(Servo.class, "outR"); //Outtake hood right

        //Sensors
        NormalizedColorSensor colore = hardwareMap.get(NormalizedColorSensor.class, "colors"); //Ball Color Sensor
        TouchSensor toccami = hardwareMap.get(TouchSensor.class, "toccami"); //Homing Touch Sensor

        //Robot Context Init
        ctx ctx = new ctx(lfD, lbD, rfD, rbD, odoParallel, odoPerp, imu);

        //Odo Thread Init
        Thread odometryThread = new Thread(() -> {
            final long periodNs = 5_000_000; // ⁓200 Hz
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

        //TeleOp Variable Init
        double speed = SPEED; //Robot Current Speed
        long levettaTime = 0, levettaWaiter = 0; //Outtake server clock
        boolean levettaBool = false; //-
        double input, output; //Turret Rotation Raw input
        double[] lastKnownQR = {-999, -999, 0}; //Last QRcode saved

        int isWrapping = 0;
        int wrapTarget = 0;

        //Turret Homing Process
        telemetry.addData("Status", "Homing Turret");
        telemetry.update();
        turretHoming(toccami, turetta);

        telemetry.addData("Status", "Robot Ready :)");
        telemetry.update();

        waitForStart();

        //Odo Thread Start
        odometryThread.start();
        timer.reset();

        while (opModeIsActive()) {
            double x, y, h;
            synchronized(pos) {
                x = pos[0];
                y = pos[1];
                h = pos[2];
            }

            //Robot Accelerator or Decelerator
            if (gamepad1.left_bumper) {
                speed  = 1; //Max Speed
            } else if (gamepad1.left_trigger >= 0.1){
                speed  = (SPEED - 0.1)*(1 - gamepad1.left_trigger) + 0.1; //Decelerator
            } else {
                speed  = SPEED; //Normal Speed
            }

            //Drive Gamepad Input Handler
            double lX = gamepad1.left_stick_x, lY = -gamepad1.left_stick_y;
            lX = Math.abs(lX) < .4 ? 0 : lX; //Left Stick X
            lY = Math.abs(lY) < .4 ? 0 : lY; //Left Stick Y
            double rX = gamepad1.right_stick_x, rY = -gamepad1.right_stick_y;
            rX = Math.abs(rX) < .2 ? 0 : rX; //Right Stick X
            rY = Math.abs(rY) < .2 ? 0 : rY; //Right Stick Y

            //Mecanum Wheels Drive
            double[] MotArr = MotorOut(lX, lY, rX, rY);
            ctx.lFd.setPower(MotArr[0] * speed);
            ctx.lBd.setPower(MotArr[1] * speed);
            ctx.rFd.setPower(MotArr[2] * speed);
            ctx.rBd.setPower(MotArr[3] * speed);

            telemetry.addData("DriveMotors", "%.2f %.2f %.2f %.2f", MotArr[0], MotArr[1], MotArr[2], MotArr[3]);

            //QR Code Auto-Aim
            input = 0; output = 0;
            if (!tagProcessor.getDetections().isEmpty()) {
                List<AprilTagDetection> tags = tagProcessor.getDetections();
                for (AprilTagDetection tag : tags) {
                    if (tag.metadata != null) { //QR code detected :)
                        double tx = tag.ftcPose.x, ty = tag.ftcPose.y; //Horizontal Distance, Forward Distance

                        if (gamepad2.triangle) { //if Auto-Aiming -> Store it
                            lastKnownQR[0] = tx;
                            lastKnownQR[1] = ty;
                            lastKnownQR[2] = 0;
                        }
                    }
                }
            } else {
                telemetry.addLine("No AprilTags detected");
            }

            //QR code storing system
            lastKnownQR[2]+=timer.milliseconds(); //Make time pass

            if (lastKnownQR[2] > QR_LIVE_TIME) lastKnownQR[0] = -999; //Kill expired QR

            if (lastKnownQR[0] != -999) {
                if (lastKnownQR[1] > 250) {
                    hoodPos = .62;
                    output = 0.87;
                }
            }

            if (Math.abs(lastKnownQR[0]) > 10 && lastKnownQR[0]!= -999) {
                double trackSpeed = lastKnownQR[2] < 100 ? Math.pow(lastKnownQR[0], 2)/lastKnownQR[1] * 0.01 : 0.01; //Get track speed with funciton V = x²/y * 0.1

                if (Math.signum(lastKnownQR[0]) == 1) {input = -Math.min(AUTOAIM_MAX_SPEED, Math.max(trackSpeed, AUTOAIM_MIN_SPEED));} //Get Tracking Direction and Normalize Raw Speed
                else {input = Math.min(AUTOAIM_MAX_SPEED, Math.max(trackSpeed, AUTOAIM_MIN_SPEED));}

            } else {
                input = 0;
            }

            //Compensate Robot Angular Velocity if AutoAiming while rotating
            if (Math.abs(rX) > 0.05 && gamepad2.triangle) {
                input = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate * ANGULAR_TO_RAW * -1;
            }

            //Main Motors Manual Handler
            if (!gamepad2.triangle) output=gamepad2.right_trigger; //Flywheel motor Manual Handler
            if (!levettaBool) in.setPower(gamepad2.left_trigger); //Intake motor Handler

            gianluca.setPower(output);

            //Hood Position Manual Handler
            if ((gamepad2.dpad_down && hoodPos > 0.47) && !gamepad2.triangle) {
                hoodPos-=0.01; //Lower
            }
            if ((gamepad2.dpad_up && hoodPos < 0.8) && !gamepad2.triangle) {
                hoodPos+=0.01; //Raise
            }
            outL.setPosition(hoodPos); //left
            outR.setPosition(1-hoodPos); //right

            //Turret Manual Handler
            if (!gamepad2.triangle) {
                if (gamepad2.left_bumper) input = 0.2;
                else if (gamepad2.right_bumper) input = -0.2;
            }

            //Outtake Server Clock Handler
            if (!gamepad2.cross) { //Reset
                levettaWaiter = System.currentTimeMillis();
            }


            if (gamepad2.cross && !levettaBool) { //Clock Init
                in.setPower(1);
                levettaTime = System.currentTimeMillis();
                levettaBool = true;
            }


            if (levettaBool) { //Stages Cycle handler
                in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                long dt = System.currentTimeMillis() - levettaTime;
                NormalizedRGBA rgb = colore.getNormalizedColors();

                if (dt < 300) {
                    if (rgb.blue > 0.001) {levetta.setPosition(.75);} //Activate only if a ball is detected
                } else if (dt < 900) {
                    levetta.setPosition(0.43);
                } else {
                    levetta.setPosition(0.43);
                    levettaBool = false;
                }

                if (System.currentTimeMillis() - levettaWaiter > 800 && dt > 300) { //Intake Sync Handler
                    in.setPower(1);
                } else {
                    in.setPower(0);
                }

            } else { //Normal Status
                levetta.setPosition(0.44);
                in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            //RAW -> FINAL Turret Handler
            int pos = turetta.getCurrentPosition();

            if (isWrapping == 0 && pos >= TURRET_MAX && input > 0) { //Upper Constraint Handler
                isWrapping = -1;
                wrapTarget = TURRET_MIN;
            }

            else if (isWrapping == 0 && pos <= TURRET_MIN && input < 0) { //Lower Constraint Handler
                isWrapping = 1;
                wrapTarget = TURRET_MAX;
            }

            if (isWrapping != 0) { //Wrapping Handler


                if ((isWrapping == -1 && input > 0) || (isWrapping == 1 && input < 0)) { //Move Target to simulate 360° rot
                    wrapTarget+= (int) (input*WRAPPING_TICKS*timer.milliseconds());
                }
                //turetta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                int d = Math.min(Math.abs(wrapTarget - pos), Math.abs(isWrapping == 1 ? pos : TURRET_MAX-pos)); //RAW Motor Power


                double wrapPower = d * 0.003; //Normalized Power Calculator
                wrapPower = Math.max(0.2, wrapPower); //Lower Bound
                wrapPower = Math.min(1, wrapPower); //Upper Bound
                wrapPower *= isWrapping == 1 ? 1 : -1; //Direction

                if ((isWrapping == 1 && pos > wrapTarget) || (isWrapping == -1 && pos < wrapTarget)) { //End sequence Observer
                    isWrapping = 0;

                    turetta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Stop and reset motor
                    turetta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                turetta.setPower(wrapPower); //Feed Processed Output

            } else { //Normal Conditions ( not Wrapping )
                if (gamepad2.triangle) turetta.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Use encoder for more precision if AutoAiming
                turetta.setPower(input);
            }

            //telemetry.addData("lra", "l: %.2f r: %.2f a: %.2f", oParallel, oPerp, oHeading);
            telemetry.addData("xyt", "x: %.2f y: %.2f t: %.2f", x, y, Math.toDegrees(h));
            telemetry.addData("loop", "%.1f ms", timer.milliseconds());

            telemetry.update();
            timer.reset();
        }

        //Odometry Thread killer
        if (odometryThread.isAlive()) {
            odometryThread.interrupt();
            odometryThread.join();
        }

        //Camera Killer
        visionPortal.close();
    }

    //Mecanum Drive
    private double[] MotorOut(double lX, double lY, double rX, double rY) {

        double h = pos[2];
        double rot;
        if (Math.abs(rX) < .05) {
            //KP Gyro-corrected straight

            if (!hLock) {
                targetH = h;
                hLock = true;
            }
            double e = angleWrap(targetH - h);
            //rot = e * KP;
            rot = 0; //TEMP
        } else {
            //Normal Rot
            hLock = false;
            rot = rX;
            targetH = h;
        }

        //Motors Raw Output
        double lf = lY + lX + rot;
        double lb = lY - lX + rot;
        double rf = lY - lX - rot;
        double rb = lY + lX - rot;

        //Normalized outputs
        double max = Math.max(1, Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb)))));
        return new double[]{lf/max, lb/max, rf/max, rb/max}; //lf, lb, rf, rb
    }

    //Threaded Odometry function
    private void odometry(ctx ctx) {
        //Odo Init
        double parallel = ctx.odoParallel.getCurrentPosition() * cmTickRatio;
        double perp = ctx.odoPerp.getCurrentPosition() * cmTickRatio;

        double imuHeading = getHeading(ctx);

        //delta values
        double dParallel = parallel - oParallel;
        double dPerp = perp - oPerp;
        double dHeading = angleWrap(imuHeading - oHeading);

        oParallel = parallel;
        oPerp = perp;
        oHeading = imuHeading;

        //Calculating Translate-only values
        double corrX = dParallel - dHeading * PaY;
        double corrY = dPerp + dHeading * PrX;
        double midHeading = pos[2] + dHeading/2;

        double cos = Math.cos(midHeading), sin = Math.sin(midHeading);

        //Thread sync
        synchronized(pos) {
            pos[0] += corrX * cos - corrY * sin;
            pos[1] += corrX * sin + corrY * cos;
            pos[2] += dHeading;
            pos[2] = angleWrap(pos[2]);
        }
    }

    //GetHeading
    private double getHeading(ctx ctx) {
        return ctx.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    //Angle Wrapper from -2π to 2π
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2*Math.PI;
        while (angle < -Math.PI) angle += 2*Math.PI;
        return angle;
    }

    //Turret Starting Alignment ( homing )
    private void turretHoming(TouchSensor toccami, DcMotor turetta) {
        while (!toccami.isPressed()) { //Wait till known position
            turetta.setPower(-0.2);
        }

        //Motor Reset and Init
        turetta.setPower(0);
        turetta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turetta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Align to Correct Offset
        turetta.setPower(0.1);
        turettaTarget = TURRET_OFFSET;

        while (turetta.getCurrentPosition() < turettaTarget) {} //Wait till alignment
        turetta.setPower(0);
    }

    //Manual Exposure camera settings
    private void setManualExposure(VisionPortal visionPortal, int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    //Robot Hardware context class
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