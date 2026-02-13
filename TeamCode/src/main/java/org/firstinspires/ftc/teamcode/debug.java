package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;

import java.util.concurrent.TimeUnit;

@TeleOp(name="FA_debug", group="Main")
public class debug extends LinearOpMode {

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
        //--- DriveMotors Init ---
        DcMotor fast = hardwareMap.get(DcMotor.class, "fast");

        Servo faster = hardwareMap.get(Servo.class, "faster");

        fast.setDirection(DcMotor.Direction.FORWARD);

        fast.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fast.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fast.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        timer.reset();

        double speed = .65;

        while (opModeIsActive()) {

            if (gamepad1.right_trigger > 0) {
                fast.setPower(gamepad1.right_trigger);
            } else {
                fast.setPower(-gamepad1.left_trigger);
            }

            if (gamepad1.cross) {
                faster.setPosition(0.5);
            }
            if (gamepad1.triangle){
                faster.setPosition(0.0);
            }
            if (gamepad1.square){
                faster.setPosition(1);
            }

            if(gamepad2.cross) {
                faster.setPosition(0.8);
            } else {
                faster.setPosition(0.44);
            }
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
        return 0.5 + 1f/150f*a*dir; //1dx -1sx
    }

    private void odometry(ctx ctx) {
        double x = pos[0], y = pos[1], t = pos[2];

        oRP = cRP; oLP = cLP; oAP = cAP;
        cRP = -ctx.e2.getCurrentPosition(); cLP = ctx.e1.getCurrentPosition(); cAP = ctx.e3.getCurrentPosition();

        int dN1 = cLP - oLP, dN2 = cRP - oRP, dN3 = cAP - oAP;

        double dT = cmTickRatio * (dN2 - dN1) / L;
        double dX = cmTickRatio * (dN1 + dN2) / 2.0;
        double dY = cmTickRatio * (dN3 - dT * B);

        double theta = t + dT/2.0;
        x += (dX * Math.cos(theta) - dY * Math.sin(theta));
        y += (dX * Math.sin(theta) + dY * Math.cos(theta));
        t += dT;

        pos[0]=x; pos[1] = y; pos[2] = t;
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