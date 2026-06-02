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

@TeleOp(name="MainTest", group="Main")
public class debug extends LinearOpMode {

    //timer obj
    ElapsedTime timer = new ElapsedTime();

    //MAIN GLOBAL CONSTANTS
    boolean DEBUGGING = true; //Debugging Const
    double SPEED = .5; //Robot Speed
    double PaY = -4.99, PrX = 9.73, R = 2, N = 8192, KP = 2; //Odometry Constants
    int QR_LIVE_TIME = 1000; //QR Code Expire time
    double CAMERA_OFFSET = 5; //Camera Offset
    double cmTickRatio = 2 * Math.PI * R / N;

    //MAIN GLOBAL VARIABLES
    final double[] pos = {0, 0, 0, 0}; //Global Robot x, y, h, Δh
    double hoodPos = .25; //Hood Position
    double shoot = 0;
    double POWER_Q = .24;
    double oParallel = 0, oPerp = 0, oHeading = 0; //Old Odometry values vars
    double[] turretLock = {-999, 0}; //Turret Lock Position
    double speed = SPEED; //Robot Current Speed
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
        DcMotor odoParallel = hardwareMap.get(DcMotor.class, "bonolis"); //Parallel Encoder
        DcMotor odoPerp = hardwareMap.get(DcMotor.class, "laZappa"); //Perpendicular Encoder

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
        DcMotor gianluca = hardwareMap.get(DcMotor.class, "gianluca"); //Flywheel bottom
        DcMotor Daroui = hardwareMap.get(DcMotor.class, "Daroui"); //Flywheel top
        DcMotor laZappa = hardwareMap.get(DcMotor.class, "laZappa"); //Front Intake
        DcMotor bonolis = hardwareMap.get(DcMotor.class, "bonolis"); //Rolling Intake

        gianluca.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Daroui.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gianluca.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Daroui.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gianluca.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Daroui.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Servo turettaL = hardwareMap.get(Servo.class, "turettaL"); //Turret Left
        Servo cecchettinR = hardwareMap.get(Servo.class, "cecchettinR"); //Turret Right
        Servo amilcare = hardwareMap.get(Servo.class, "amilcare"); //Levetta
        Servo carlR = hardwareMap.get(Servo.class, "carlR"); //Outtake hood right
        Servo marxL = hardwareMap.get(Servo.class, "marxL"); //Outtake hood left

        //Robot Context Init
        ctx ctx = new ctx(lfD, lbD, rfD, rbD, odoParallel, odoPerp, imu);

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

            //Mecanum Wheels Drive
            double[] MotArr = MotorOut(lX, lY, rX, rY);
            ctx.lFd.setPower(MotArr[0] * speed);
            ctx.lBd.setPower(MotArr[1] * speed);
            ctx.rFd.setPower(MotArr[2] * speed);
            ctx.rBd.setPower(MotArr[3] * speed);

            telemetry.addData("DriveMotors", "%.2f %.2f %.2f %.2f", MotArr[0], MotArr[1], MotArr[2], MotArr[3]);

            //QR Code Auto-Aim
            double input = 0; double output = 0; //Turret Rotation Raw input, Flywheel output, Min Turret Rotation Speed
            if (!tagProcessor.getDetections().isEmpty() && gamepad2.triangle) {
                List<AprilTagDetection> tags = tagProcessor.getDetections();
                for (AprilTagDetection tag : tags) {
                    if (tag.metadata != null) { //QR code detected :)
                        double tx = tag.ftcPose.x, ty = tag.ftcPose.y, r=tag.ftcPose.yaw;//Horizontal Distance, Forward Distance
                        //Store QR
                        lastKnownQR[0] = tx;
                        lastKnownQR[1] = ty;
                        lastKnownQR[2] = 0;
                        lastKnownQR[3] = r;
                    }
                }
            } else {
                telemetry.addLine("No AprilTags detected");
            }

            //QR code storing system
            lastKnownQR[2]+=timer.milliseconds(); //Make time pass

            if (lastKnownQR[2] > QR_LIVE_TIME) lastKnownQR[0] = -999; //Kill expired QR

            if (lastKnownQR[0] != -999 && gamepad2.dpad_left) {
                output = (lastKnownQR[1]/100)/7 + POWER_Q;
                hoodPos = 0.25;
            }

            telemetry.addData("out", output);
            telemetry.addData("dist", lastKnownQR[1]);

            //Autoaim at QR code
            double qrOffset = lastKnownQR[0] - (48 * Math.cos(Math.PI/2 - Math.toRadians(lastKnownQR[3])) - CAMERA_OFFSET);
            //

            //Main Motors Manual Handler
            if (!gamepad2.triangle) {
                output = gamepad2.right_trigger; //Flywheel motor Manual Handler
            }
            if (!gamepad2.cross) {
                laZappa.setPower(gamepad2.left_trigger); //Intake motor Handler
                bonolis.setPower(gamepad2.left_trigger > 0.1 ? 0.3 : 0);
                amilcare.setPosition(.3);
                shoot = 0;
            } else {
                laZappa.setPower(0.4);
                bonolis.setPower(shoot < 400 ? 0.3 : 1);
                amilcare.setPosition(0);

                shoot+=timer.milliseconds();
            }
            telemetry.addData("shoot", shoot);

            gianluca.setPower(output);
            Daroui.setPower(output);

            //Hood Position Manual Handler
            if ((gamepad2.dpad_down && hoodPos > 0) && !gamepad2.triangle) {
                hoodPos-=0.002*timer.milliseconds(); //Lower
            }
            if ((gamepad2.dpad_up && hoodPos < 0.6) && !gamepad2.triangle) {
                hoodPos+=0.002*timer.milliseconds(); //Raise
            }

            marxL.setPosition(hoodPos); //left
            carlR.setPosition(1-(hoodPos)); //right

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