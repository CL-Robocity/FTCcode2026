package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name="AutoFrameWork", group="Main")
public class auto extends LinearOpMode {

    //timer obj
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime timer3 = new ElapsedTime();

    //MAIN GLOBAL CONSTANTS
    boolean DEBUGGING = false; //Debugging Const
    double SPEED = .5; //Robot Speed
    double PaY = -4.99, PrX = 9.73, R = 2, N = 8192, KP = 2; //Odometry Constants
    int TURRET_OFFSET = 1320; //Turret Starting Position
    int TURRET_MAX = 2600, TURRET_MIN = -70; //Turret Constraints
    double AUTOAIM_MIN_SPEED = 0.01, AUTOAIM_MAX_SPEED = 0.2; //Auto-Aiming Speed
    int QR_LIVE_TIME = 2000; //QR Code Expire time
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
    int levettaBool = 0;
    double[] lastKnownQR = {-999, -999, 0, 0}; //Last QRcode saved


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

        setManualExposure(visionPortal, 2, 200);
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

        turetta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turetta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo levetta = hardwareMap.get(Servo.class, "levetta"); //Outtake server
        Servo outL = hardwareMap.get(Servo.class, "outL"); //Outtake hood left
        Servo outR = hardwareMap.get(Servo.class, "outR"); //Outtake hood right

        //Sensors
        NormalizedColorSensor colore = hardwareMap.get(NormalizedColorSensor.class, "colors"); //Ball Color Sensor
        TouchSensor toccami = hardwareMap.get(TouchSensor.class, "toccami"); //Homing Touch Sensor

        //Robot Context Init
        ctx ctx = new ctx(lfD, lbD, rfD, rbD, odoParallel, odoPerp, gianluca, in, turetta, outL, outR, levetta, imu, tagProcessor, colore);

        waitForStart();
        gianluca.setPower(0.8);

        tRawPos = TURRET_OFFSET+200;
        while(turretMovement(turetta, TURRET_OFFSET+200, 0.1)) {idle();};

        shoot(ctx, 7000, 1000);

        straight(ctx, 60, 0.6, false, false);
        sleep(500);

        align(ctx, 90, 0.3);

        straight(ctx, 90, 0.3, true, false);

        align(ctx, 60, 0.3);

        straight(ctx, -90, 0.6, false, true);

        tRawPos = TURRET_OFFSET-400;
        while(turretMovement(turetta, TURRET_OFFSET-400, 0.1)) {idle();};

        shoot(ctx, 7000, 1000);

        straight(ctx, 50, 1, false, true);

        timer.reset();

        while(turretMovement(turetta, 0, 0.1)) {idle();};
        //Camera Killer
        visionPortal.close();
    }

    private void straight(ctx ctx, double cm, double maxSpeed, boolean intake, boolean flywheel) {
        double cmToTicks = 384.5 / (Math.PI * 10.4); // Ticks per cm
        double targetTicks = cm * cmToTicks;
        double kP = 0.004;      // Proportional gain: How "hard" it pushes to reach the target
        double tolerance = 15;  // Ticks of allowed error (roughly 0.5cm)
        double minSpeed = 0.12; // Minimum power to overcome friction/static weight

        //Reset Encoders
        ctx.lFd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ctx.lBd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ctx.rBd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ctx.rFd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ctx.lFd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ctx.lBd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ctx.rBd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ctx.rFd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            int currentPos = ctx.lFd.getCurrentPosition();
            double error = targetTicks - currentPos;

            //Exit condition
            if (Math.abs(error) < tolerance) break;

            //Calculate Power (Error * Gain)
            double p = error * kP;

            //Apply "Floor" and "Ceiling" to speed
            double signedMin = Math.signum(p) * minSpeed;
            if (Math.abs(p) < minSpeed) p = signedMin;

            p = Math.max(-maxSpeed, Math.min(p, maxSpeed));

            ctx.lFd.setPower(p);
            ctx.lBd.setPower(p);
            ctx.rFd.setPower(p);
            ctx.rBd.setPower(p);

            if (intake) ctx.in.setPower(1);
            if (flywheel) ctx.gianluca.setPower(0.7);

            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Current Ticks", currentPos);
            telemetry.addData("Power", p);
            telemetry.update();
        }

        // 6. Hard stop
        ctx.lFd.setPower(0);
        ctx.lBd.setPower(0);
        ctx.rFd.setPower(0);
        ctx.rBd.setPower(0);
        ctx.in.setPower(0);
    }

    private void align(ctx ctx, double targetDeg, double maxSpeed) {
        double tolerance = 1.0; // How close is "good enough"
        double kP = 0.015;      // Increased strength (0.005 was too low)
        double minPower = 0.2; // Minimum power to overcome carpet friction

        while (opModeIsActive()) {
            double currentAngle = degreeWrap(ctx.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            //Calculate shortest distance to target
            double error = degreeWrap(targetDeg - currentAngle);

            // Exit if we are within tolerance
            if (Math.abs(error) <= tolerance) break;

            //Proportional calculation
            double motorPower = error * kP;

            //Add minPower so it doesn't stall near the target
            if (Math.abs(motorPower) < minPower) {
                motorPower = Math.signum(motorPower) * minPower;
            }

            //Cap at maxSpeed
            motorPower = Math.max(-maxSpeed, Math.min(motorPower, maxSpeed));

            //Apply to motors
            ctx.lFd.setPower(-motorPower);
            ctx.lBd.setPower(-motorPower);
            ctx.rFd.setPower(motorPower);
            ctx.rBd.setPower(motorPower);

            telemetry.addData("Target", targetDeg);
            telemetry.addData("Current", currentAngle);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        // Brake
        ctx.lFd.setPower(0);
        ctx.lBd.setPower(0);
        ctx.rFd.setPower(0);
        ctx.rBd.setPower(0);
    }

    private double degreeWrap(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private void shoot(ctx ctx, double ms, double delay) {
        boolean triangle = true, dpad_left = true;
        timer2.reset();
        timer3.reset();
        double hoodError = 0, outputError = 1;
        while (opModeIsActive() && timer2.milliseconds() < ms) {
            boolean cross = timer2.milliseconds() > delay;
            double input = 0; double output = 0, minTurretSpeed = 0.1; //Turret Rotation Raw input, Flywheel output, Min Turret Rotation Speed

            if (!ctx.tagProcessor.getDetections().isEmpty() && triangle) {
                List<AprilTagDetection> tags = ctx.tagProcessor.getDetections();
                for (AprilTagDetection tag : tags) {
                    if (tag.metadata != null) { //QR code detected :)
                        double tx = tag.ftcPose.x, ty = tag.ftcPose.y, r=tag.ftcPose.yaw;//Horizontal Distance, Forward Distance
                        //Store QR
                        lastKnownQR[0] = tx;
                        lastKnownQR[1] = ty;
                        lastKnownQR[2] = 0;
                        lastKnownQR[3] = r;

                        ERR = 5;
                    }
                }
            } else {
                telemetry.addLine("No AprilTags detected");
                ERR = 10;
            }

            //QR code storing system
            lastKnownQR[2]+=timer3.milliseconds(); //Make time pass

            if (lastKnownQR[2] > QR_LIVE_TIME) lastKnownQR[0] = -999; //Kill expired QR

            if (lastKnownQR[0] != -999 && dpad_left) {
                output = (lastKnownQR[1]/100)/7 + 0.36;

                if (lastKnownQR[1] < 250) {
                    hoodPos = .53;
                    output+= 0.05;
                } else {
                    hoodPos = .6;
                }
            }

            //Autoaim at QR code
            double qrOffset = lastKnownQR[0] - (48 * Math.cos(Math.PI/2 - Math.toRadians(lastKnownQR[3])) - CAMERA_OFFSET);

            if (Math.abs(qrOffset) > 10 && lastKnownQR[2] < 500) {
                double trackSpeed = dpad_left ? Math.pow(qrOffset, 2)/(2*lastKnownQR[1]) * 0.01 : 0; //Get track speed with funciton V = x²/y * 0.1

                if (Math.signum(qrOffset) == 1) {input = -Math.min(AUTOAIM_MAX_SPEED, Math.max(trackSpeed, AUTOAIM_MIN_SPEED));} //Get Tracking Direction and Normalize Raw Speed
                else {input = Math.min(AUTOAIM_MAX_SPEED, Math.max(trackSpeed, AUTOAIM_MIN_SPEED));}

                minTurretSpeed = AUTOAIM_MIN_SPEED;
            } else {
                input = 0;
            }

            ctx.gianluca.setPower(output * outputError);
            ctx.outL.setPosition(hoodPos - hoodError); //left
            ctx.outR.setPosition(1-(hoodPos - hoodError)); //right

            //Outtake Server Clock Handler
            if (!cross) { //Reset
                levettaWaiter = System.currentTimeMillis();
            }

            if (cross && levettaBool == 0) { //Clock Init
                ctx.in.setPower(1);
                levettaTime = System.currentTimeMillis();
                levettaBool = 1;
            }

            if (levettaBool > 0) { //Stages Cycle handler
                ctx.in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                long dt = System.currentTimeMillis() - levettaTime;
                NormalizedRGBA rgb = ctx.colore.getNormalizedColors();

                telemetry.addData("rgb", rgb.blue);
                telemetry.addData("dt", dt);

                if (dt < 500) { //Stage1: Push up to "ready" postition
                    if (rgb.blue > 0.001) { //Activate only if a ball is detected
                        ctx.levetta.setPosition(.61);
                        levettaBool = 2;
                    }
                } else if (dt < 700) { //Stage2: SHOOT
                    if (levettaBool == 2) ctx.levetta.setPosition(.75);
                } else if (dt < 1500) { //Stage3: Retreat
                    ctx.levetta.setPosition(0.43);
                } else { //Stage4: Wait
                    levettaBool = 0;
                }

                if (System.currentTimeMillis() - levettaWaiter > 800 && dt > 700) { //Intake Sync Handler
                    ctx.in.setPower(1);
                    hoodError = .05;
                    outputError = 1.07;
                } else {
                    ctx.in.setPower(0);
                }

            } else { //Intake servo rest position
                ctx.levetta.setPosition(0.43);
                ctx.in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            //Turret Raw Position Handler
            tRawPos += input * POWER_TO_TICKS * timer3.milliseconds();
            turretLock[1] += input * POWER_TO_TICKS * timer3.milliseconds();

            //Turret Handler
            turretMovement(ctx.turetta, tRawPos, minTurretSpeed);

            hoodError = hoodError - timer.milliseconds()/600*0.015 < 0 ? 0 : hoodError - timer.milliseconds()/600*0.015;
            outputError = outputError - timer.milliseconds()/600*0.04 < 1 ? 1 : outputError - timer.milliseconds()/600*0.04;

            telemetry.update();
            timer3.reset();
        }
        ctx.gianluca.setPower(0);
    }

    //Mecanum Drive
    private double[] MotorOut(double lX, double lY, double rX, double rY) {
        double rot = rX;

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
    private boolean turretMovement(DcMotor turetta, double tRawPos, double minSpeed) {
        double range = Math.abs(TURRET_MAX - TURRET_MIN); //Movement Range

        double tPos = ((tRawPos + Math.abs(TURRET_MIN))%range + range) % range - Math.abs(TURRET_MIN); //tPos Normalized Position
        double c = turetta.getCurrentPosition(); //Turret Current Position

        double err = Math.abs(tPos - c); //Error

        if (err > ERR) { //Move turret
            int dir = tPos > c ? 1 : -1; //Get direction

            double d = Math.min(err, dir == 1 ? Math.abs(c - TURRET_MIN) : Math.abs(c - TURRET_MAX));//RAW Motor Power

            double p = Math.max(Math.min(d * TURRET_ACCEL, 1), minSpeed) * dir; //Normalized Power Calculator

            turetta.setPower(p);
            return true;
        } else {
            turetta.setPower(0);
            oTurret = c;
            return false;
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
        while (angle > 2*Math.PI) angle -= 2*Math.PI;
        while (angle < 0) angle += 2*Math.PI;
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
        turetta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        public final DcMotor gianluca;
        public final DcMotor in;
        public final DcMotor turetta;
        public final Servo outL;
        public final Servo outR;
        public final Servo levetta;
        public final IMU imu; //Inertial Mesurement Unit
        public final AprilTagProcessor tagProcessor;
        public final NormalizedColorSensor colore;

        public ctx(DcMotor lFd,
                   DcMotor lBd,
                   DcMotor rFd,
                   DcMotor rBd,
                   DcMotor odoParallel,
                   DcMotor odoPerp,
                   DcMotor gianluca,
                   DcMotor in,
                   DcMotor turetta,
                   Servo outL,
                   Servo outR,
                   Servo levetta,
                   IMU imu,
                   AprilTagProcessor tagProcessor,
                   NormalizedColorSensor colore
        ) {
            this.lFd = lFd;
            this.lBd = lBd;
            this.rFd = rFd;
            this.rBd = rBd;
            this.odoParallel = odoParallel;
            this.odoPerp = odoPerp;
            this.gianluca = gianluca;
            this.in = in;
            this.turetta = turetta;
            this.outL = outL;
            this.outR = outR;
            this.levetta = levetta;
            this.imu = imu;
            this.tagProcessor = tagProcessor;
            this.colore = colore;
        }
    }
}