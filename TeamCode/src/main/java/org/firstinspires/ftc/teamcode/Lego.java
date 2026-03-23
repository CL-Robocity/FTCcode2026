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

@TeleOp(name="Slego", group="Main")
public class Lego extends LinearOpMode {

    //timer obj
    ElapsedTime timer = new ElapsedTime();

    //MAIN GLOBAL CONSTANTS
    boolean DEBUGGING = true; //Debugging Const
    double SPEED = .5; //Robot Speed
    double QR_LIVE_TIME = 50; //QR live time

    //MAIN GLOBAL VARIABLES
    double speed = SPEED;
    double[] lastKnownQR = {0, 0, 0, 0};
    double config = 0;
    boolean check = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //Dashboard Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "ASPETTA UN ATTIMO");
        telemetry.update();

        //Camera Init
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTag(21, "QR", 41, DistanceUnit.CM)
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

        //setManualExposure(visionPortal, 1, 150);
        if (DEBUGGING) FtcDashboard.getInstance().startCameraStream(visionPortal, 24);

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

        //Robot Context Init
        ctx ctx = new ctx(lfD, lbD, rfD, rbD);

        telemetry.addData("Status", "Robot Ready :)");
        telemetry.update();

        waitForStart();

        timer.reset();

        while (opModeIsActive()) {

            //Robot Accelerator or Decelerator
            if (gamepad1.left_bumper) {
                speed  = 1; //Max Speed
            } else if (gamepad1.left_trigger >= 0.1){
                speed  = (SPEED - 0.1)*(1 - gamepad1.left_trigger) + 0.1; //Decelerator
            } else {
                speed  = SPEED; //Normal Speed
            }

            if (gamepad1.cross && !check) {
                config = config == 1 ? 0 : 1;
                check = true;
            } else if (!gamepad1.cross){
                check = false;
            }

            if (config == 0) {
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
            } else {
                double lY = 0, rX = 0;
                if (lastKnownQR[0] != -999) {

                    if (lastKnownQR[1] > 150 && lastKnownQR[1] <= 200) {
                        lY = -0.8;
                    } else if (lastKnownQR[1] > 200) {
                        lY = -1;
                        speed = 1;
                    } else if (lastKnownQR[1] < 120) {
                        lY = 0.8;
                    }
                    if (lastKnownQR[1] > 6000/lastKnownQR[1]) {
                        speed = SPEED;
                        rX = -lastKnownQR[0]*0.02;
                    }
                }
                double[] MotArr = MotorOut(0, lY, rX, 0);
                ctx.lFd.setPower(MotArr[0] * speed);
                ctx.lBd.setPower(MotArr[1] * speed);
                ctx.rFd.setPower(MotArr[2] * speed);
                ctx.rBd.setPower(MotArr[3] * speed);

                telemetry.addData("DriveMotors", "%.2f %.2f %.2f %.2f", MotArr[0], MotArr[1], MotArr[2], MotArr[3]);
            }

            //QR Code Auto-Aimdouble input = 0; double output = FLYWHEEL_MINSPEED, minTurretSpeed = 0.1; //Turret Rotation Raw input, Flywheel output, Min Turret Rotation Speed
            if (!tagProcessor.getDetections().isEmpty()) {
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

            if (lastKnownQR[0] != -999) telemetry.addData("Qrcode", "X: %.1f, Y: %.1f, YAW: %.1f", lastKnownQR[0], lastKnownQR[1], lastKnownQR[2]);
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

        public ctx(DcMotor lFd, DcMotor lBd, DcMotor rFd, DcMotor rBd) {
            this.lFd = lFd;
            this.lBd = lBd;
            this.rFd = rFd;
            this.rBd = rBd;
        }
    }
}