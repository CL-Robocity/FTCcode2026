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
    boolean DEBUGGING = false; //Debugging Const
    double SPEED = .5; //Robot Speed
    double PaY = -4.99, PrX = 9.73, R = 2, N = 8192, KP = 2; //Odometry Constants
    int TURRET_OFFSET = 1320; //Turret Starting Position
    int TURRET_MAX = 2600, TURRET_MIN = -70; //Turret Constraints
    double AUTOAIM_MIN_SPEED = 0.05, AUTOAIM_MAX_SPEED = 0.2; //Auto-Aiming Speed
    int QR_LIVE_TIME = 500; //QR Code Expire time
    double CAMERA_OFFSET = 13; //Camera Offset
    double RAD_TO_TICKS = 1325/Math.PI; //Turret Angle to Motor Ticks
    double POWER_TO_TICKS = 3.5; //Motor Power to Turret Ticks
    double TURRET_ACCEL = 0.001; //Turret Acceleration
    double cmTickRatio = 2 * Math.PI * R / N;

    //MAIN GLOBAL VARIABLES
    final double[] pos = {0, 0, 0, 0}; //Global Robot x, y, h, Δh
    int turettaTarget = 90; //Homing temp var
    boolean hLock = false;
    double targetH = 0; //Target Heading
    double hoodPos = .5; //Hood Position
    double tRawPos = TURRET_OFFSET;
    double oParallel = 0, oPerp = 0, oHeading = 0, oTurret = TURRET_OFFSET; //Old Odometry values vars, Old Turret Pos
    double[] turretLock = {-999, 0}; //Turret Lock Position

    @Override
    public void runOpMode() throws InterruptedException {
        //Dashboard Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "ASPETTA UN ATTIMO");
        telemetry.update();

        //IMU Init
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        );

        imu.initialize(parameters);

        telemetry.addData("Status", "Calibrating IMU");
        sleep(1000);
        idle();

        imu.resetYaw();

        //Camera Init
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

        setManualExposure(visionPortal, 1, 200);
        if (DEBUGGING) FtcDashboard.getInstance().startCameraStream(visionPortal, 24);

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

        //TeleOp Variable Init
        double speed = SPEED; //Robot Current Speed
        long levettaTime = 0, levettaWaiter = 0; //Outtake server clock
        int levettaBool = 0;
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

        timer.reset();

        while (opModeIsActive()) {
            odometry(ctx);
            double x = pos[0], y = pos[1], h = pos[2];

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

            if (Math.abs(lX) > 0.05 || Math.abs(lY) > 0.05) {TURRET_ACCEL = 0.003;}

            //Mecanum Wheels Drive
            double[] MotArr = MotorOut(lX, lY, rX, rY, h);
            ctx.lFd.setPower(MotArr[0] * speed);
            ctx.lBd.setPower(MotArr[1] * speed);
            ctx.rFd.setPower(MotArr[2] * speed);
            ctx.rBd.setPower(MotArr[3] * speed);

            telemetry.addData("DriveMotors", "%.2f %.2f %.2f %.2f", MotArr[0], MotArr[1], MotArr[2], MotArr[3]);

            //QR Code Auto-Aim
            double input = 0; double output = 0, minTurretSpeed = 0.1; //Turret Rotation Raw input, Flywheel output, Min Turret Rotation Speed
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
                if (lastKnownQR[1] > 250) { //Far AutoPower
                    hoodPos = .62;
                    output = 0.87;
                }
            }

            if (gamepad2.square) { //test Autopower
                output = 0.6;
                hoodPos = .65;
            }

            //Autoaim at QR code
            double qrOffset = lastKnownQR[0] + CAMERA_OFFSET;
            if (Math.abs(qrOffset) > 10 && lastKnownQR[0]!= -999) {
                double trackSpeed = lastKnownQR[2] < 100 ? Math.pow(qrOffset, 2)/lastKnownQR[1] * 0.01 : 0.01; //Get track speed with funciton V = x²/y * 0.1

                if (Math.signum(qrOffset) == 1) {input = -Math.min(AUTOAIM_MAX_SPEED, Math.max(trackSpeed, AUTOAIM_MIN_SPEED));} //Get Tracking Direction and Normalize Raw Speed
                else {input = Math.min(AUTOAIM_MAX_SPEED, Math.max(trackSpeed, AUTOAIM_MIN_SPEED));}

                minTurretSpeed = AUTOAIM_MIN_SPEED;
            } else {
                input = 0;
            }

            //Accelerate turret if rotating
            if (gamepad2.triangle && Math.abs(rX) > 0.05) {
                TURRET_ACCEL = 0.005;
            } else {
                TURRET_ACCEL = 0.001;
            }

            //Main Motors Manual Handler
            if (!gamepad2.triangle && !gamepad2.square) output=gamepad2.right_trigger; //Flywheel motor Manual Handler
            if (levettaBool == 0) in.setPower(gamepad2.left_trigger); //Intake motor Handler

            gianluca.setPower(output);

            //Hood Position Manual Handler
            if ((gamepad2.dpad_down && hoodPos > 0.47) && (!gamepad2.triangle && !gamepad2.square)) {
                hoodPos-=0.002*timer.milliseconds(); //Lower
            }
            if ((gamepad2.dpad_up && hoodPos < 0.8) && (!gamepad2.triangle && !gamepad2.square)) {
                hoodPos+=0.002*timer.milliseconds(); //Raise
            }
            outL.setPosition(hoodPos); //left
            outR.setPosition(1-hoodPos); //right

            //Turret Manual Handler
            if (!gamepad2.triangle) {
                if (gamepad2.left_bumper) {input = 0.2; minTurretSpeed = 0.2;}
                else if (gamepad2.right_bumper) {input = -0.2; minTurretSpeed = 0.2;}
            }

            //Outtake Server Clock Handler
            if (!gamepad2.cross) { //Reset
                levettaWaiter = System.currentTimeMillis();
            }

            if (gamepad2.cross && levettaBool == 0) { //Clock Init
                in.setPower(1);
                levettaTime = System.currentTimeMillis();
                levettaBool = 1;
            }

            if (levettaBool > 0) { //Stages Cycle handler
                in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                long dt = System.currentTimeMillis() - levettaTime;
                NormalizedRGBA rgb = colore.getNormalizedColors();

                if (dt < 300) { //Stage1: Push up to "ready" postition
                    if (rgb.blue > 0.005) { //Activate only if a ball is detected
                        levetta.setPosition(.65);
                        levettaBool = 2;
                    } else if (rgb.blue > 0.001) { //Little movement to move the ball
                        levetta.setPosition(.5);
                        dt=800;
                    }
                } else if (dt < 700) { //Stage2: SHOOT
                    if (levettaBool == 2) levetta.setPosition(.78);
                } else if (dt < 900) { //Stage3: Retreat
                    levetta.setPosition(0.43);
                } else if (dt < 2000) { //Stage4: Wait
                    levettaBool = 0;
                }

                if (System.currentTimeMillis() - levettaWaiter > 800 && dt > 700) { //Intake Sync Handler
                    in.setPower(1);
                } else {
                    in.setPower(0);
                }

            } else { //Intake servo rest position
                levetta.setPosition(0.44);
                in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            //Turret Raw Position Handler
            tRawPos += input * POWER_TO_TICKS * timer.milliseconds();
            turretLock[1] += input * POWER_TO_TICKS * timer.milliseconds();

            //Compensate Robot Angular Velocity if AutoAiming while rotating
            if (gamepad2.triangle) {
                if (turretLock[0] == -999) {
                    turretLock[0] = h;
                    turretLock[1] = turetta.getCurrentPosition();
                }

                tRawPos = turretLock[1] - angleWrap(h - turretLock[0])*RAD_TO_TICKS;
            } else {
                turretLock[0] = -999;
            }

            //Turret Handler
            turretMovement(turetta, tRawPos, minTurretSpeed);

            telemetry.addData("xyt", "x: %.2f y: %.2f t: %.2f", x, y, Math.toDegrees(h));
            telemetry.addData("loop", "%.1f ms", timer.milliseconds());

            telemetry.update();
            idle();
            sleep(5); //Delay the loops
            timer.reset();
        }

        //Camera Killer
        visionPortal.close();
    }

    //Mecanum Drive
    private double[] MotorOut(double lX, double lY, double rX, double rY, double h) {
        double rot;
        if (Math.abs(rX) < .05) {

            if (!hLock) {
                targetH = h;
                hLock = true;
            }
            double e = angleWrap(targetH - h);
            rot = 0;//e * KP;
        } else {
            //Normal Rot
            hLock = false;
            rot = rX;
            targetH = h;
        }

        //Motors Raw Output
        double y = lY/*lY*Math.cos(h)+lX*Math.sin(h)*/, x = lX/*lX*Math.cos(h)-lY*Math.sin(h)*/;

        double lf = y + x + rot;
        double lb = y - x + rot;
        double rf = y - x - rot;
        double rb = y + x - rot;

        //Normalized outputs
        double max = Math.max(1, Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb)))));
        return new double[]{lf/max, lb/max, rf/max, rb/max}; //lf, lb, rf, rb
    }

    //Turret Handler
    private void turretMovement(DcMotor turetta, double tRawPos, double minSpeed) {
        double range = Math.abs(TURRET_MAX - TURRET_MIN); //Movement Range

        double tPos = ((tRawPos + Math.abs(TURRET_MIN))%range + range) % range - Math.abs(TURRET_MIN); //tPos Normalized Position
        double c = turetta.getCurrentPosition(); //Turret Current Position

        double err = Math.abs(tPos - c); //Error

        if (err > 10) { //Move turret
            int dir = tPos > c ? 1 : -1; //Get direction

            double d = Math.min(err, dir == 1 ? Math.abs(c - TURRET_MIN) : Math.abs(c - TURRET_MAX));//RAW Motor Power

            double p = Math.max(Math.min(d * TURRET_ACCEL, 1), minSpeed) * dir; //Normalized Power Calculator

            turetta.setPower(p);
        } else {
            turetta.setPower(0);
            oTurret = c;
        }
    }

    //Threaded Odometry function
    private void odometry(ctx ctx) {
        //Odo Init
        double parallel = ctx.odoParallel.getCurrentPosition() * cmTickRatio;
        double perp = ctx.odoPerp.getCurrentPosition() * cmTickRatio;

        double imuHeading;
        try { //Get IMU heading
            imuHeading = ctx.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        } catch (Exception e) {
            imuHeading = oHeading; // use last valid value
        }

        //delta values
        double dParallel = parallel - oParallel;
        double dPerp = perp - oPerp;
        double dHeading = angleWrap(imuHeading - oHeading);

        //Old Values
        oParallel = parallel;
        oPerp = perp;
        oHeading = imuHeading;

        //Calculating Translate-only values
        double corrX = dParallel - dHeading * PaY;
        double corrY = dPerp + dHeading * PrX;

        double midHeading = oHeading + dHeading/2; //Avg Heading MID rotation

        double cos = Math.cos(midHeading), sin = Math.sin(midHeading);

        //Odometry
        pos[0] += corrX * cos - corrY * sin;
        pos[1] += corrX * sin + corrY * cos;
        pos[2] = angleWrap(imuHeading);
    }

    //Angle Wrapper from -2π to 2π
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2*Math.PI;
        while (angle < -Math.PI) angle += 2*Math.PI;
        return angle;
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
        turetta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Move to offset position
        turetta.setPower(0.1);
        while (!isStopRequested() && turetta.getCurrentPosition() < TURRET_OFFSET) {
            idle();
        }
        turetta.setPower(0);
    }

    //Manual Exposure camera settings
    private void setManualExposure(VisionPortal visionPortal, int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        //Wait till camera is loaded
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        //Set Manual Exposure and Gain
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