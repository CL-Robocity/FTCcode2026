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
    double CAMERA_OFFSET = 0;
    double cmTickRatio = 2 * Math.PI * R / N;
    double KP_FACTOR = 1.0;

    final double[] pos = {0, 0, 0, 0};
    double hoodPos = .25;
    double shootTime = 0;
    double POWER_Q = .25;
    boolean lastDpadUp2 = false;
    boolean lastDpadDown2 = false;
    double oParallel = 0, oPerp = 0, oHeading = 0;
    double speed = SPEED;
    double[] lastKnownQR = {-999, -999, 0, 0};
    double TurretPosition = 0.5;
    double TURRET_KP = 0.002;
    double output = 0;
    // variabili in game

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

        // reset encoder delle odo
        odoParallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoParallel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // riconoscimento motori di movimento
        DcMotor lfD = hardwareMap.get(DcMotor.class, "lf");
        DcMotor lbD = hardwareMap.get(DcMotor.class, "lb");
        DcMotor rfD = hardwareMap.get(DcMotor.class, "rf");
        DcMotor rbD = hardwareMap.get(DcMotor.class, "rb");

        // set direzione motori
        lfD.setDirection(DcMotor.Direction.REVERSE);
        lbD.setDirection(DcMotor.Direction.REVERSE);
        rfD.setDirection(DcMotor.Direction.FORWARD);
        rbD.setDirection(DcMotor.Direction.FORWARD);

        // set per runnare i motori
        lfD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set movimento a potenza 0
        lfD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotorEx TopFlyWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "gianluca");
        DcMotorEx DownFlyWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Daroui");
        DcMotor FrontIntake = hardwareMap.get(DcMotor.class, "laZappa");
        DcMotor IntakeRoller = hardwareMap.get(DcMotor.class, "bonolis");

        TopFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DownFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TopFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DownFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        TopFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DownFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients currentPIDF = TopFlyWheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients aggressivePIDF = new PIDFCoefficients(currentPIDF.p * KP_FACTOR, currentPIDF.i, currentPIDF.d, currentPIDF.f);
        TopFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, aggressivePIDF);
        DownFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, aggressivePIDF);

        telemetry.addData("Motors: ", "Ready :)");

        Servo LeftTurretServo = hardwareMap.get(Servo.class, "turettaL");
        Servo RightTurretServo = hardwareMap.get(Servo.class, "cecchettinR");
        Servo BallStopServo = hardwareMap.get(Servo.class, "amilcare");
        Servo RampLeftServo = hardwareMap.get(Servo.class, "marxL");
        Servo CoverRightServo = hardwareMap.get(Servo.class, "carlR");

        telemetry.addData("Servo: ", "Ready :)");

        ctx ctx = new ctx(lfD, lbD, rfD, rbD, odoParallel, odoPerp, imu);

        telemetry.addData("ctx: ", "Ready :)");
        telemetry.addData("\nStatus", "Robot Ready :)");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            odometry(ctx);
            double x = pos[0], y = pos[1], h = pos[2];

            // set motori flywheel
            LeftTurretServo.setPosition(TurretPosition);
            RightTurretServo.setPosition(TurretPosition);

            /*//gestione velocità movimento
            if (gamepad1.left_bumper) {
                speed = 1; // l1  --> velocità massima --> potenza 1
            } else if (gamepad1.left_trigger >= 0.1) {
                speed = (SPEED - 0.1) * (1 - gamepad1.left_trigger) + 0.1; // l2 --> frenata
            } else {
                speed = SPEED; // velocità standard di sistema
            }

            // set joystick sinistro
            double lX = gamepad1.left_stick_x, lY = -gamepad1.left_stick_y;
            if(gamepad1.dpad_up) lY = 1;
            if(gamepad1.dpad_down) lY = -1;
            if(gamepad1.dpad_right) lX = 1;
            if(gamepad1.dpad_left) lX = -1;
            lX = Math.abs(lX) < .4 ? 0 : lX;
            lY = Math.abs(lY) < .4 ? 0 : lY;

            // set joystick destro
            double rX = gamepad1.right_stick_x, rY = -gamepad1.right_stick_y;
            rX = Math.abs(rX) < .2 ? 0 : rX;
            rY = Math.abs(rY) < .2 ? 0 : rY;

            //definizione array motori e output movimento motori
            double[] MotArr = MotorOut(lX, lY, rX, rY);
            ctx.lFd.setPower(MotArr[0] * speed);
            ctx.lBd.setPower(MotArr[1] * speed);
            ctx.rFd.setPower(MotArr[2] * speed);
            ctx.rBd.setPower(MotArr[3] * speed);




            // Alzi la potenza della flywheel con la freccetta IN SU
            if (gamepad2.dpad_right && !lastDpadUp2) {
                POWER_Q += 0.02;
                if (POWER_Q > 1.0) {
                    POWER_Q = 1.0; // Limite massimo di sicurezza per i motori
                }
            }
            lastDpadUp2 = gamepad2.dpad_right; // Memorizza lo stato corrente per il prossimo ciclo


            // Abbassi la potenza della flywheel con la freccetta IN GIÙ ---
            if (gamepad2.dpad_left && !lastDpadDown2) {
                POWER_Q -= 0.02;
                if (POWER_Q < 0.0) {
                    POWER_Q = 0.0; // Evita che vada in negativo
                }
            }
            lastDpadDown2 = gamepad2.dpad_left; // Memorizza lo stato

            // detect qr code
            if (!tagProcessor.getDetections().isEmpty() && gamepad2.triangle) { //se ha detectato qualcosa e triangolo premuto
                List<AprilTagDetection> tags = tagProcessor.getDetections();
                for (AprilTagDetection tag : tags) {
                    if (tag.metadata != null && tag.metadata.id == 24) {
                        lastKnownQR[0] = tag.ftcPose.x; // allineamento in orizzontale
                        lastKnownQR[1] = tag.ftcPose.y; // distanza dal qr code
                        lastKnownQR[2] = 0; // timer dal detect dell'ultimo qr code
                        lastKnownQR[3] = tag.ftcPose.yaw;

                        telemetry.addData("qr", "x: %.2f, y: %.2f, t: %.2f, yaw: %.2f", lastKnownQR[0], lastKnownQR[1], lastKnownQR[2], lastKnownQR[3]);
                    }
                }
            }
            // set potenza in output
            lastKnownQR[2] += timer.milliseconds();
            if (lastKnownQR[2] > QR_LIVE_TIME) lastKnownQR[0] = -999;

            if (lastKnownQR[0] != -999 && gamepad2.triangle) {
                if (lastKnownQR[1] > 200) {
                    output = (lastKnownQR[1] / 100) / 7 + POWER_Q;
                    hoodPos = shootTime > 700 ? 0.50 : 0.58; // dopo : prima
                } else {
                    output = 0.3 + POWER_Q;
                    hoodPos = shootTime > 300 ? 0.5 : 0.54; // dopo : prima
                }
                if (shootTime > 700) output += 0.04;
            }

            // se non stai premendo triangolo va in base al R2
             if (!gamepad2.triangle) {
               output = Math.max(gamepad2.right_trigger, 0.5);

            }

            double targetVelocity = output * 2500;
            TopFlyWheel.setVelocity(targetVelocity);
            DownFlyWheel.setVelocity(targetVelocity);

            // set potenza dell'intake bilaterale per entrambi i pad
            double intake = gamepad1.right_trigger > gamepad2.left_trigger ? Math.min(1, gamepad1.right_trigger + 0.1) : Math.min(1, gamepad2.left_trigger + 0.1);

            // se non stai premendo la croce --> servo di stop non completamente fuori
            if (!gamepad2.cross) {
                FrontIntake.setPower(intake);
                IntakeRoller.setPower(intake > 0.1 ? 0.3 : 0);
                BallStopServo.setPosition(.25);
                shootTime = 0;
            } else {// se x è premuta
                double currentVel = Math.abs(TopFlyWheel.getVelocity());
                boolean flywheelReady = (targetVelocity > 100) && (currentVel >= (targetVelocity - 150));

                if (shootTime > 200) {
                    FrontIntake.setPower(.4);
                    IntakeRoller.setPower(1);
                } else {
                    FrontIntake.setPower(0);
                    IntakeRoller.setPower(0);
                }
                BallStopServo.setPosition(0);
                shootTime += timer.milliseconds();
            }

            // set posizione servo hood
            if ((gamepad2.dpad_down && hoodPos > 0) && !gamepad2.triangle) {
                hoodPos -= 0.002 * timer.milliseconds();
            }
            if ((gamepad2.dpad_up && hoodPos < 0.6) && !gamepad2.triangle) {
                hoodPos += 0.002 * timer.milliseconds();
            }

            // aggiorna posizione hood
            RampLeftServo.setPosition(hoodPos);
            CoverRightServo.setPosition(1 - hoodPos);

            // autoaim oppure aim manuale
            if (gamepad2.triangle && lastKnownQR[0] != -999) {
                double qrOffset = lastKnownQR[0] - (48 * Math.cos(Math.PI/2 - Math.toRadians(lastKnownQR[3])) - CAMERA_OFFSET);
                double distance = lastKnownQR[1];// + Math.abs(48 * Math.sin(Math.PI/2 - Math.toRadians(lastKnownQR[3])));
                telemetry.addData("qrOffset", qrOffset);
                telemetry.addData("distance", 48 * Math.sin(Math.PI/2 - Math.toRadians(lastKnownQR[3])));

                double theta = Math.atan(qrOffset / distance);
                telemetry.addData("angle", Math.toDegrees(theta));

                TurretPosition = (Math.toDegrees(theta) * 2.63 + 165) / 300;

                telemetry.addData("tempPos", TurretPosition);
            } else {
                if (gamepad2.right_bumper && !gamepad2.left_bumper) { // right bumper a destra
                    TurretPosition += 0.001 * timer.milliseconds();
                }
                if (gamepad2.left_bumper && !gamepad2.right_bumper) { // left bumper a sinistra
                    TurretPosition -= 0.001 * timer.milliseconds();
                }
            }

            // limite torretta (90° totali = ±45°)
            TurretPosition = Math.max(0.02, Math.min(0.98, TurretPosition));


            telemetry.addData("TurretPos",LeftTurretServo.getPosition());

            telemetry.addData("HoodPos", RampLeftServo.getPosition());

            telemetry.addData("xyt", "x: %.2f y: %.2f t: %.2f", x, y, Math.toDegrees(h));
            telemetry.addData("Flywheel Target", targetVelocity);
            telemetry.addData("Flywheel Actual", TopFlyWheel.getVelocity());

            telemetry.addData("\n DistanceQR", lastKnownQR[1]);

            telemetry.addData("\n ODO PARALLEL --> ", odoParallel.getCurrentPosition());
            telemetry.addData("ODO PERP --> ", odoPerp.getCurrentPosition());

            telemetry.addData("\n Fly Wheel Power : ", POWER_Q);*/

            telemetry.addData("\n Pointing deg : ", Math.toDegrees(Math.atan2(-gamepad1.left_stick_y,gamepad1.left_stick_x)));
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