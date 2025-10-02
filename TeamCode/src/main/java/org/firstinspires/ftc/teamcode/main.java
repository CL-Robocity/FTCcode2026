package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="MainDrive", group="Main")
public class main extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();

    int cRP, oRP, cLP, oLP, cAP, oAP; //current position, old position -> Left, Right, Aux

    //173,5 180,5
    double L = 17.7, B = 8.9, R = 2, N = 8192;
    double cmTickRatio = 2 * Math.PI * R / N;
    double[] pos = {0, 0, 0};

    @Override
    public void runOpMode() {
        //--- Dashboard Init ---
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //---

        //--- Camera Init ---
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
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

        lfD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lfD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //---

        //--- Motors and Servos Init ---
        /*
        DcMotor dcMotor = hardwareMap.get(DcMotor.class, "DcMotorName");
        Servo servo = hardwareMap.get(Servo.class, "servoName");

        dcMotor.setDirection(DcMotor.Direction.FORWARD);
        dcMotor.setTargetPosition(0);
        dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */
        //---

        //--- Odometry Encoders Init ---
        DcMotor odoL = hardwareMap.get(DcMotor.class, "odo1");
        DcMotor odoR = hardwareMap.get(DcMotor.class, "odo2");
        DcMotor odoA = hardwareMap.get(DcMotor.class, "odo3");
        odoL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //---

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Robot Context Init
        ctx ctx = new ctx(lfD, lbD, rfD, rbD, odoL, odoR, odoA);

        waitForStart();
        timer.reset();

        double speed = .7;
        while (opModeIsActive()) {
            odometry(ctx);

            double x = pos[0], y = pos[1], t = pos[2];

            double lX = gamepad1.left_stick_x, lY = -gamepad1.left_stick_y;
            lX = Math.abs(lX) < .4 ? 0 : lX; //dead zone
            lY = Math.abs(lY) < .4 ? 0 : lY;
            double rX = gamepad1.right_stick_x, rY = -gamepad1.right_stick_y;
            rX = Math.abs(rX) < .2 ? 0 : rX;
            rY = Math.abs(rY) < .2 ? 0 : rY;

            double[] MotArr = MotorOut(-lX, -lY, rX, rY);

            ctx.lFd.setPower(MotArr[0] * speed);
            ctx.lBd.setPower(MotArr[1] * speed);
            ctx.rFd.setPower(MotArr[2] * speed);
            ctx.rBd.setPower(MotArr[3] * speed);

            if (!tagProcessor.getDetections().isEmpty()) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("CameraTag", "x: %.2f y: %.2f z: %.2f roll: %.2f pitch: %.2f yaw: %.2f",  tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z, tag.ftcPose.roll, tag.ftcPose.pitch, tag.ftcPose.yaw);
            }

            telemetry.addData("lra", "l: %6d r: %6d a: %6d", cLP, cRP, cAP);
            telemetry.addData("xyt", "x: %.2f y: %.2f t: %.2f", x, y, Math.toDegrees(t));
            telemetry.addData("loop", "%.1f ms", timer.milliseconds());
            telemetry.update();
            timer.reset();
        }
    }

    private double[] MotorOut(double lX, double lY, double rX, double rY) {
        if (rX == 0) {
            double a = Math.atan2(lY, lX), p = Math.hypot(lX, lY);

            double sin = Math.sin(a - Math.PI / 4), cos = Math.cos(a - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double o1 = p * cos / max, o2 = p * sin / max;

            //lf, lb, rf, rb
            return new double[]{o1, o2, o2, o1};
        } else {
            //Rotation ->
            return new double[]{rX, rX, -rX, -rX};
        }
    }

    private double getAng(double a, int dir) {
        return 0.5 + 1f/135f*a*dir; //1dx -1sx
    }

    private void odometry(ctx ctx) {
        double x = pos[0], y = pos[1], t = pos[2];

        oRP = cRP; oLP = cLP; oAP = cAP;
        cRP = -ctx.e2.getCurrentPosition(); cLP = -ctx.e1.getCurrentPosition(); cAP = ctx.e3.getCurrentPosition();

        int dN1 = cLP - oLP, dN2 = cRP - oRP, dN3 = cAP - oAP;

        double dT = cmTickRatio * (dN2 - dN1) / L;
        double dX = cmTickRatio * (dN1 + dN2) / 2.0;
        double dY = cmTickRatio * (dN3 - (dN2 - dN1) * B / L);

        double theta = t + dT/2.0;
        x += dX * Math.cos(theta) - dY * Math.sin(theta);
        y += dX * Math.sin(theta) + dY * Math.cos(theta);
        t += dT;

        pos[0]=x; pos[1] = y; pos[2] = t;
    }

    static class ctx {
        public final DcMotor lFd;
        public final DcMotor lBd;
        public final DcMotor rFd;
        public final DcMotor rBd;
        public final DcMotor e1;
        public final DcMotor e2;
        public final DcMotor e3;

        public ctx(DcMotor lFd, DcMotor lBd, DcMotor rFd, DcMotor rBd, DcMotor e1, DcMotor e2, DcMotor e3) {
            this.lFd = lFd;
            this.lBd = lBd;
            this.rFd = rFd;
            this.rBd = rBd;
            this.e1 = e1;
            this.e2 = e2;
            this.e3 = e3;
        }
    }
}