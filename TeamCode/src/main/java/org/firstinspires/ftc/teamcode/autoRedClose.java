package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

@Autonomous(name="Red Close__Auto", group="Main")
public class autoRedClose extends LinearOpMode {

    //timer obj
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime timer3 = new ElapsedTime();

    //MAIN GLOBAL CONSTANTS
    boolean DEBUGGING = false;
    double SPEED = .5;
    double PaY = -4.99, PrX = 9.73, R = 2, N = 8192, KP = 2;
    int QR_LIVE_TIME = 1000;
    double CAMERA_OFFSET = 0;
    double cmTickRatio = 2 * Math.PI * R / N;
    double KP_FACTOR = 1.0;

    final double[] pos = {0, 0, 0, 0};
    double hoodPos = .25;
    double shootTime = 0;
    double POWER_Q = .27;
    double oParallel = 0, oPerp = 0, oHeading = 0;
    double speed = SPEED;
    double[] lastKnownQR = {-999, -999, 0, 0};
    double TurretPosition = 0.55;
    double TURRET_KP = 0.002;
    double output = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //Dashboard Init
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
                .addTag(23, "PPG", 41, DistanceUnit.CM)
                .addTag(21, "GPP", 41, DistanceUnit.CM)
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

        DcMotor odoParallel = hardwareMap.get(DcMotor.class, "roller");
        DcMotor odoPerp = hardwareMap.get(DcMotor.class, "frontintake");

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

        DcMotorEx TopFlyWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "topfly");
        DcMotorEx DownFlyWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "bottomfly");
        DcMotor FrontIntake = hardwareMap.get(DcMotor.class, "frontintake");
        DcMotor IntakeRoller = hardwareMap.get(DcMotor.class, "roller");

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

        Servo LeftTurretServo = hardwareMap.get(Servo.class, "turretleft");
        Servo RightTurretServo = hardwareMap.get(Servo.class, "turretright");
        Servo BallStopServo = hardwareMap.get(Servo.class, "ballstop");
        Servo RampLeftServo = hardwareMap.get(Servo.class, "rampleft");
        Servo CoverRightServo = hardwareMap.get(Servo.class, "coverright");

        telemetry.addData("Servo: ", "Ready :)");

        ctx ctx = new ctx(
                lfD,
                lbD,
                rfD,
                rbD,
                odoParallel,
                odoPerp,
                imu,
                TopFlyWheel,
                DownFlyWheel,
                FrontIntake,
                IntakeRoller,
                LeftTurretServo,
                RightTurretServo,
                BallStopServo,
                RampLeftServo,
                CoverRightServo,
                tagProcessor);

        telemetry.addData("ctx: ", "Ready :)");
        telemetry.addData("\nStatus", "Robot Ready :)");
        telemetry.update();

        waitForStart();

        RightTurretServo.setPosition(0.5);
        LeftTurretServo.setPosition(0.5);


        sleep(500);
        straight(ctx, 130, 180, 0.4, true , false);
        autoShoot(ctx,4000);
        sleep(500);
        align(ctx, -30, 0.3, false);
        sleep(200);
        straight(ctx, 30, -90, 0.2, false, false);



        timer.reset();

        //Camera Killer
        visionPortal.close();
    }







    /**
     * Muove il robot in qualsiasi direzione a 360° senza ruotare il muso.
     */
    private void straight(ctx ctx, double targetCm, double moveAngleDeg, double speed, boolean flywheel, boolean intake) {
        ElapsedTime moveTimer = new ElapsedTime();

        // KP per la correzione (se oscilla ancora un po' dopo il fix, scendilo a 1.2)
        double KP_ALIGN = 1.5;

        double moveAngleRad = Math.toRadians(moveAngleDeg);
        double decel_distance = 15.0;
        double distace_error = 2.0;

        double driveDirection = Math.cos(moveAngleRad);
        double strafeDirection = Math.sin(moveAngleRad);

        double startX = pos[0];
        double startY = pos[1];
        double targetHeading = pos[2];

        moveTimer.reset();

        while (opModeIsActive()) {
            odometry(ctx);

            if (flywheel) {
                double baseVelocity = 0.45 * 2500;
                ctx.topFly.setVelocity(baseVelocity);
                ctx.downFly.setVelocity(baseVelocity);
            } else {
                ctx.topFly.setVelocity(0);
                ctx.downFly.setVelocity(0);
            }

            if (intake) {
                ctx.inRoller.setPower(0.7);
                ctx.frontIn.setPower(0.7);
            } else {
                ctx.inRoller.setPower(0);
                ctx.frontIn.setPower(0);
            }

            double deltaX = pos[0] - startX;
            double deltaY = pos[1] - startY;
            double distanceTraveled = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            double errorCm = targetCm - distanceTraveled;

            if (errorCm <= distace_error) {
                break;
            }

            double magnitude = Math.min(1.0, errorCm / decel_distance);

            double driveInput = driveDirection * magnitude;
            double strafeInput = strafeDirection * magnitude;

            // =================================================================
            // CORREZIONE APPLICATA: Adesso contrasta l'errore invece di amplificarlo
            // =================================================================
            double headingError = angleWrap(targetHeading - pos[2]);
            double turnInput = -headingError * KP_ALIGN;

            double lfRaw = driveInput + strafeInput + turnInput;
            double lbRaw = driveInput - strafeInput + turnInput;
            double rfRaw = driveInput - strafeInput - turnInput;
            double rbRaw = driveInput + strafeInput - turnInput;

            double lfOut = lfRaw * speed;
            double lbOut = lbRaw * speed;
            double rfOut = rfRaw * speed;
            double rbOut = rbRaw * speed;

            double maxMotorOut = Math.max(1.0, Math.max(
                    Math.max(Math.abs(lfOut), Math.abs(lbOut)),
                    Math.max(Math.abs(rfOut), Math.abs(rbOut))
            ));

            ctx.lFd.setPower(lfOut / maxMotorOut);
            ctx.lBd.setPower(lbOut / maxMotorOut);
            ctx.rFd.setPower(rfOut / maxMotorOut);
            ctx.rBd.setPower(rbOut / maxMotorOut);

            telemetry.addData("=== OMNI-STRAIGHT ===", "ATTIVO");
            telemetry.addData("Distanza Percorsa", "%.1f / %.1f cm", distanceTraveled, targetCm);
            telemetry.addData("Errore Muso (deg)", "%.2f°", Math.toDegrees(headingError));
            telemetry.update();

            idle();
        }
        // deact del movement
        ctx.lFd.setPower(0);
        ctx.lBd.setPower(0);
        ctx.rFd.setPower(0);
        ctx.rBd.setPower(0);

        // deact di intake
        ctx.inRoller.setPower(0);
        ctx.frontIn.setPower(0);

    }

    /**
     * Ruota il robot sul posto fino a raggiungere l'angolo target specificato.
     */
    private void align(ctx ctx, double targetAngleDeg, double maxPower, boolean flywheel) {
        ElapsedTime turnTimer = new ElapsedTime();

        double targetHeadingRad = Math.toRadians(targetAngleDeg);
        double KP_TURN = 2.0;
        double ALLOWED_ERROR_RAD = Math.toRadians(1.0);

        turnTimer.reset();

        while (opModeIsActive() && turnTimer.seconds() < 3.0) {
            odometry(ctx);

            // flywheel activation
            if (flywheel) {
                double baseVelocity = 0.45 * 2500;
                ctx.topFly.setVelocity(baseVelocity);
                ctx.downFly.setVelocity(baseVelocity);
            } else {
                ctx.topFly.setVelocity(0);
                ctx.downFly.setVelocity(0);
            }

            // Calcola l'errore angolare simmetrico [-PI, PI]
            double headingError = angleWrap(targetHeadingRad - pos[2]);

            if (Math.abs(headingError) <= ALLOWED_ERROR_RAD) {
                break;
            }

            double turnPower = headingError * KP_TURN;

            if (turnPower > maxPower)  turnPower = maxPower;
            if (turnPower < -maxPower) turnPower = -maxPower;

            if (Math.abs(turnPower) < 0.12) {
                turnPower = Math.signum(turnPower) * 0.12;
            }

            double lfPower = -turnPower;
            double lbPower = -turnPower;
            double rfPower = turnPower;
            double rbPower = turnPower;

            ctx.lFd.setPower(lfPower);
            ctx.lBd.setPower(lbPower);
            ctx.rFd.setPower(rfPower);
            ctx.rBd.setPower(rbPower);

            telemetry.addData("=== ALLINEAMENTO ANGOLO ===", "ROTAZIONE");
            telemetry.addData("Angolo Target (deg)", targetAngleDeg);
            telemetry.addData("Angolo Corrente (deg)", Math.toDegrees(pos[2]));
            telemetry.addData("Errore Rimanente (deg)", Math.toDegrees(headingError));
            telemetry.update();

            idle();
        }

        ctx.lFd.setPower(0);
        ctx.lBd.setPower(0);
        ctx.rFd.setPower(0);
        ctx.rBd.setPower(0);
    }

    /**
     * UNICO METODO DI WRAP: Gestisce in modo simmetrico i radianti nell'intervallo [-PI, PI].
     * Fondamentale sia per l'odometria (dHeading) che per il calcolo degli errori di sterzo.
     */
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Riproduce fedelmente il comportamento della TeleOp:
     * - Primi 500ms: aggancia il tag, muove la torretta e l'hood, accelera i volani (Simula TRIANGOLO).
     * - Successivamente: apre il ballstop e avvia i motori di carico con 200ms di ritardo (Simula TRIANGOLO + X).
     */
    private void autoShoot(ctx ctx, double durationMs) {
        ElapsedTime shootTimer = new ElapsedTime(); // Timer totale del metodo
        ElapsedTime loopTimer = new ElapsedTime();  // Timer per calcolare il delta time (dt)

        // Variabili di stato locali identiche alla TeleOp
        double shootTime = 0;
        double output = 0;
        double hoodPos = 0.25;

        shootTimer.reset();
        loopTimer.reset();

        while (opModeIsActive() && shootTimer.milliseconds() < durationMs) {
            // Calcola il tempo trascorso dall'ultimo ciclo (dt) per simulare il timer della TeleOp
            double dt = loopTimer.milliseconds();
            loopTimer.reset();

            odometry(ctx);

            // 1. RILEVAMENTO APRILTAG (Esattamente come dentro "if (gamepad2.triangle)")
            if (!ctx.tagProcessor.getDetections().isEmpty()) {
                List<AprilTagDetection> tags = ctx.tagProcessor.getDetections();
                for (AprilTagDetection tag : tags) {
                    if (tag.metadata != null && tag.metadata.id == 24) {
                        lastKnownQR[0] = tag.ftcPose.x;
                        lastKnownQR[1] = tag.ftcPose.y;
                        lastKnownQR[2] = 0;
                        lastKnownQR[3] = tag.ftcPose.yaw;
                    }
                }
            }

            // Aggiorna il ciclo di vita del target QR
            lastKnownQR[2] += dt;
            if (lastKnownQR[2] > QR_LIVE_TIME) lastKnownQR[0] = -999;

            // 2. CALCOLO POTENZA FLYWHEEL E POSIZIONE HOOD
            if (lastKnownQR[0] != -999) {
                if (lastKnownQR[1] > 200) {
                    output = (lastKnownQR[1] / 100) / 7 + POWER_Q;
                    hoodPos = shootTime > 700 ? 0.50 : 0.58;
                } else {
                    output = 0.3 + POWER_Q;
                    hoodPos = shootTime > 700 ? 0.50 : 0.56;
                }
                if (shootTime > 700) output += 0.04;
            } else {
                // Fallback di sicurezza se non vede il tag all'inizio
                output = 0.3 + POWER_Q;
            }

            double targetVelocity = output * 2500;
            ctx.topFly.setVelocity(targetVelocity);
            ctx.downFly.setVelocity(targetVelocity);

            // 3. AUTO-AIM DELLA TORRETTA
            if (lastKnownQR[0] != -999) {
                double qrOffset = lastKnownQR[0] - (48 * Math.cos(Math.PI/2 - Math.toRadians(lastKnownQR[3])) - CAMERA_OFFSET);
                double distance = lastKnownQR[1];

                double theta = Math.atan(qrOffset / distance);
                TurretPosition = (Math.toDegrees(theta) * 2.63 + 165) / 300;
            }

            // Limiti fisici torretta
            TurretPosition = Math.max(0.02, Math.min(0.98, TurretPosition));
            ctx.turretL.setPosition(TurretPosition);
            ctx.turretR.setPosition(TurretPosition);

            // Aggiorna i servomotori dell'hood
            ctx.rampL.setPosition(hoodPos);
            ctx.coverR.setPosition(1 - hoodPos);

            // 4. TIMING DI SPARO (La tua sequenza temporale)
            if (shootTimer.milliseconds() < 500) {
                // FASE A: Primi 500ms -> Solo mira e calcolo (Triangolo ON, X OFF)
                ctx.frontIn.setPower(0);
                ctx.inRoller.setPower(0);
                ctx.ballStop.setPosition(0.25); // Cancello chiuso
                shootTime = 0;
            } else {
                // FASE B: Dopo 500ms -> Inizia la sequenza di fuoco (Triangolo ON + X ON)
                ctx.ballStop.setPosition(0); // Il cursore si apre e RESTA aperto

                // Gestione del ritardo dei 200ms per i motori di carico copiato dalla TeleOp
                if (shootTime > 200) {
                    ctx.frontIn.setPower(0.4);
                    ctx.inRoller.setPower(1.0);
                } else {
                    ctx.frontIn.setPower(0);
                    ctx.inRoller.setPower(0);
                }
                shootTime += dt; // Incrementa il tempo di sparo attivo (fondamentale per le soglie >200 e >700)
            }

            // Telemetria di controllo in tempo reale
            telemetry.addData("=== AUTO SHOOT ===", "REPLICA TELEOP");
            telemetry.addData("Stato Pulsanti", shootTimer.milliseconds() < 500 ? "[Δ]" : "[Δ + X]");
            telemetry.addData("ShootTime Attivo (ms)", "%.0f", shootTime);
            telemetry.addData("Distanza Target", lastKnownQR[1]);
            telemetry.addData("Velocità Volano", "Target: %.0f | Real: %.0f", targetVelocity, Math.abs(ctx.topFly.getVelocity()));
            telemetry.addData("Inclinazione Hood", hoodPos);
            telemetry.update();

            idle();
        }

        // Spegnimento totale al termine del tempo impostato nell'autonoma
        ctx.topFly.setVelocity(0);
        ctx.downFly.setVelocity(0);
        ctx.frontIn.setPower(0);
        ctx.inRoller.setPower(0);
        ctx.ballStop.setPosition(0.25); // Richiudi il cancello per la fase successiva
    }

    //Mecanum Drive
    private double[] MotorOut(double lX, double lY, double rX, double rY) {
        double rot = rX;
        double y = lY, x = lX;

        double lf = y + x + rot;
        double lb = y - x + rot;
        double rf = y - x - rot;
        double rb = y + x - rot;

        double max = Math.max(1, Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb)))));
        return new double[]{lf/max, lb/max, rf/max, rb/max};
    }

    // Threaded Odometry function
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

        // FISSAATO: Ora usa l'angleWrap corretto a [-PI, PI]. I delta piccoli rimangono piccoli!
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
        pos[2] = angleWrap(imuHeading); // Mantiene anche la pos globale nel range corretto
    }

    // Manual Exposure camera settings
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

        if (!isStopRequested()) {
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

    // Robot Hardware context class
    static class ctx {
        public final DcMotor lFd;
        public final DcMotor lBd;
        public final DcMotor rFd;
        public final DcMotor rBd;
        public final DcMotor odoParallel;
        public final DcMotor odoPerp;
        public final IMU imu;
        public final DcMotorEx topFly, downFly;
        public final DcMotor frontIn, inRoller;
        public final Servo turretL, turretR, ballStop, rampL, coverR;
        public final AprilTagProcessor tagProcessor;

        public ctx(DcMotor lFd, DcMotor lBd, DcMotor rFd, DcMotor rBd, DcMotor odoParallel, DcMotor odoPerp, IMU imu,
                   DcMotorEx topFly, DcMotorEx downFly, DcMotor frontIn, DcMotor inRoller,
                   Servo turretL, Servo turretR, Servo ballStop, Servo rampL, Servo coverR,
                   AprilTagProcessor tagProcessor) {
            this.lFd = lFd;
            this.lBd = lBd;
            this.rFd = rFd;
            this.rBd = rBd;
            this.odoParallel = odoParallel;
            this.odoPerp = odoPerp;
            this.imu = imu;
            this.topFly = topFly;
            this.downFly = downFly;
            this.frontIn = frontIn;
            this.inRoller = inRoller;
            this.turretL = turretL;
            this.turretR = turretR;
            this.ballStop = ballStop;
            this.rampL = rampL;
            this.coverR = coverR;
            this.tagProcessor = tagProcessor;
        }
    }
}