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

@Disabled
@Autonomous(name="AutoFrameWork", group="Main")
public class auto extends LinearOpMode {

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


        timer.reset();

        //Camera Killer
        visionPortal.close();
    }













    /**
     * Muove il robot in qualsiasi direzione a 360° senza ruotare il muso,
     * sfruttando la cinematica Mecanum e la logica motorOut della TeleOp.
     *
     * @param ctx Il contesto hardware del robot
     * @param targetCm La distanza assoluta da percorrere in centimetri (sempre positiva)
     * @param moveAngleDeg Angolo di movimento rispetto al robot (0°=Avanti, 90°=Destra, 180°=Indietro, 270°=Sinistra)
     * @param speed Moltiplicatore generale della velocità dei motori [0.0, 1.0]
     * @param flywheel Se true, accende la flywheel standard di TeleOp
     * @param intake Se true, accende l'intake standard di TeleOp
     */
    private void straight(ctx ctx, double targetCm, double moveAngleDeg, double speed, boolean flywheel, boolean intake) {
        ElapsedTime moveTimer = new ElapsedTime();

        // CORREZIONE LOGICA 1: KP per i GRADI (evita che lo sterzo saturi subito la potenza)
        double KP_ALIGN = 0.025;

        // Convertiamo l'angolo di movimento in Radianti per le funzioni Math
        double moveAngleRad = Math.toRadians(moveAngleDeg);

        // distanza di decelerazione
        double decel_distance = 15.0;

        // tolleranza nell'arrivo: errore nell'arrivo
        double distace_error = 2;

        // Scomposizione del vettore di movimento a 360 gradi
        double driveDirection = Math.cos(moveAngleRad);  // Componente Avanti/Indietro
        double strafeDirection = Math.sin(moveAngleRad); // Componente Destra/Sinistra

        // Registrazione della posizione di partenza assoluta dall'odometria
        double startX = pos[0];
        double startY = pos[1];
        double targetHeading = pos[2]; // Il muso del robot rimarrà fisso in questa direzione

        moveTimer.reset();

        while (opModeIsActive()) {
            odometry(ctx);

            // 1. GESTIONE SISTEMI AUSILIARI (flywheel e intake)
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

            // 2. CALCOLO DELLA DISTANZA LINEARE ASSOLUTA (Pitagora)
            double deltaX = pos[0] - startX;
            double deltaY = pos[1] - startY;
            double distanceTraveled = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            double errorCm = targetCm - distanceTraveled;

            // Condizione di arrivo (tolleranza 1.0 cm)
            if (errorCm <= distace_error) {
                break;
            }

            // 3. CALCOLO DELLA MAGNITUDINE DEL MOVIMENTO
            // Mantiene la rampa di decelerazione fluida negli ultimi 15cm
            double magnitude = Math.min(1.0, errorCm / decel_distance);

            // Generazione degli input virtuali per la trazione
            double driveInput = driveDirection * magnitude;
            double strafeInput = strafeDirection * magnitude;

            // 4. CORREZIONE ANGOLARE (Mantiene il robot dritto mentre trasla)
            double headingError = angleWrap(targetHeading - pos[2]); // Assumendo restituisca Gradi [-180, 180]
            double turnInput = headingError * KP_ALIGN;

            // =================================================================
            // CINEMATICA MECANUM COMPLETA (Identica al motorOut della TeleOp)
            // =================================================================

            // Calcolo raw dei canali miscelando Drive (Y), Strafe (X) e Turn (R)
            double lfRaw = driveInput + strafeInput + turnInput;
            double lbRaw = driveInput - strafeInput + turnInput;
            double rfRaw = driveInput - strafeInput - turnInput;
            double rbRaw = driveInput + strafeInput - turnInput;

            // CORREZIONE REFUSO: Applicazione uniforme del moltiplicatore 'speed'
            double lfOut = lfRaw * speed;
            double lbOut = lbRaw * speed;
            double rfOut = rfRaw * speed;
            double rbOut = rbRaw * speed;

            // Normalizzazione dei motori per non saturare oltre 1.0 (REV Clip)
            double maxMotorOut = Math.max(1.0, Math.max(
                    Math.max(Math.abs(lfOut), Math.abs(lbOut)),
                    Math.max(Math.abs(rfOut), Math.abs(rbOut))
            ));

            // Invio definitivo della potenza normalizzata ai motori
            ctx.lFd.setPower(lfOut / maxMotorOut);
            ctx.lBd.setPower(lbOut / maxMotorOut);
            ctx.rFd.setPower(rfOut / maxMotorOut);
            ctx.rBd.setPower(rbOut / maxMotorOut);

            // =================================================================

            // Telemetria per i test sul campo
            telemetry.addData("=== OMNI-STRAIGHT ===", "ATTIVO");
            telemetry.addData("Direzione Target (Angolo)", "%.1f°", moveAngleDeg);
            telemetry.addData("Distanza Percorsa", "%.1f / %.1f cm", distanceTraveled, targetCm);
            telemetry.addData("Errore Allineamento Muso", "%.2f°", headingError);
            telemetry.update();

            idle();
        }

        // Frenata di sicurezza (ZeroPowerBehavior.BRAKE farà il resto se impostato nel robotConfig)
        ctx.lFd.setPower(0);
        ctx.lBd.setPower(0);
        ctx.rFd.setPower(0);
        ctx.rBd.setPower(0);
    }







    /**
     * Ruota il robot sul posto fino a raggiungere l'angolo target specificato.
     * @param ctx Il contesto hardware del robot
     * @param targetAngleDeg L'angolo obiettivo in gradi (es. 0 per il fronte, 90 per sinistra, -90 per destra)
     * @param maxPower La potenza massima applicabile ai motori durante la rotazione [0.1, 1.0]
     */
    private void align(ctx ctx, double targetAngleDeg, double maxPower) {
        ElapsedTime turnTimer = new ElapsedTime();

        // Convertiamo il target in radianti, dato che la vostra odometria probabilmente lavora in radianti
        double targetHeadingRad = Math.toRadians(targetAngleDeg);

        // Guadagno proporzionale per la rotazione.
        // Se il robot va lungo (overshoot), abbassatelo (es. 1.5). Se si ferma prima, alzatelo (es. 2.5).
        double KP_TURN = 2.0;

        // Tolleranza di precisione: il ciclo si interrompe quando l'errore è inferiore a 1 grado
        double ALLOWED_ERROR_RAD = Math.toRadians(1.0);

        turnTimer.reset();

        // Timeout di sicurezza (es. 3 secondi) per evitare che il robot rimanga bloccato nel ciclo se oscilla
        while (opModeIsActive() && turnTimer.seconds() < 3.0) {
            // Aggiorna la cinematica per leggere l'angolo corrente pos[2]
            odometry(ctx);

            // Calcola l'errore angolare mancante applicando l'angleWrap per evitare il problema del salto +-180°
            double headingError = angleWrap(targetHeadingRad - pos[2]);

            // Se l'errore è sotto la tolleranza, abbiamo raggiunto l'orientamento desiderato
            if (Math.abs(headingError) <= ALLOWED_ERROR_RAD) {
                break;
            }

            // --- CALCOLO POTENZA PROPORZIONALE ---
            // Più il robot è vicino all'angolo corretto, più rallenta per non scivolare
            double turnPower = headingError * KP_TURN;

            // Limitiamo la potenza al valore massimo deciso dal programmatore
            if (turnPower > maxPower)  turnPower = maxPower;
            if (turnPower < -maxPower) turnPower = -maxPower;

            // Soglia minima di potenza: sotto a 0.12 i motori Mecanum potrebbero non far muovere il robot per l'attrito
            if (Math.abs(turnPower) < 0.12) {
                turnPower = Math.signum(turnPower) * 0.12;
            }

            // --- APPLICAZIONE POTENZA AI MOTORI ---
            // Per girare sul posto, i motori dello stesso lato devono andare nella stessa direzione,
            // ma in direzione opposta rispetto al lato opposto.
            // turnPower positivo (errore positivo) = rotazione a sinistra
            double lfPower = -turnPower;
            double lbPower = -turnPower;
            double rfPower = turnPower;
            double rbPower = turnPower;

            // Invia i comandi ai moduli REV
            ctx.lFd.setPower(lfPower);
            ctx.lBd.setPower(lbPower);
            ctx.rFd.setPower(rfPower);
            ctx.rBd.setPower(rbPower);

            // Telemetria di controllo
            telemetry.addData("=== ALLINEAMENTO ANGOLO ===", "ROTAZIONE");
            telemetry.addData("Angolo Target (deg)", targetAngleDeg);
            telemetry.addData("Angolo Corrente (deg)", Math.toDegrees(pos[2]));
            telemetry.addData("Errore Rimanente (deg)", Math.toDegrees(headingError));
            telemetry.update();

            idle();
        }

        // FRENATA: Spegne i motori e azzera il movimento
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







    /**
     * Esegue il puntamento automatico tramite AprilTag (ID 24) e spara per il tempo specificato.
     * @param durationMs Durata totale dell'azione di sparo in millisecondi (es. 3000 per svuotare il caricatore)
     */
    private void autoShoot(ctx ctx, double durationMs) {
        ElapsedTime shootTimer = new ElapsedTime(); // Timer totale dello sparo
        ElapsedTime loopTimer = new ElapsedTime();  // Timer per calcolare il delta-time del loop

        double shootTime = 0; // Tiene traccia di quanto tempo la siringa sta spingendo
        double currentTurretPos = 0.55; // Posizione di partenza della torretta (neutra)

        double[] localQR = {-999, -999, 0, 0}; // x, y, age, yaw
        double POWER_Q = 0.27;
        double CAMERA_OFFSET = 0;

        shootTimer.reset();
        loopTimer.reset();

        while (opModeIsActive() && shootTimer.milliseconds() < durationMs) {
            double dt = loopTimer.milliseconds();
            loopTimer.reset();

            // 1. SCANSIONE APRILTAG (Simula la pressione costante di Triangolo)
            if (!ctx.tagProcessor.getDetections().isEmpty()) {
                List<AprilTagDetection> tags = ctx.tagProcessor.getDetections();
                for (AprilTagDetection tag : tags) {
                    if (tag.metadata != null && tag.metadata.id == 24) {
                        localQR[0] = tag.ftcPose.x;   // Offset orizzontale
                        localQR[1] = tag.ftcPose.y;   // Distanza
                        localQR[2] = 0;               // Reset dell'età del dato
                        localQR[3] = tag.ftcPose.yaw; // Orientamento tag
                    }
                }
            }

            // Invecchiamento del dato se il tag viene perso momentaneamente
            localQR[2] += dt;
            if (localQR[2] > 1000) localQR[0] = -999; // Se passa più di 1 secondo, cancella il target

            // 2. CALCOLO POTENZA FLYWHEEL E FILTRO HOOD (Dalla tua TeleOp)
            double flyOutput = 0;
            double targetHoodPos = 0.25;

            if (localQR[0] != -999) {
                // Calcolo dinamico basato sulla distanza
                if (localQR[1] > 200) {
                    flyOutput = (localQR[1] / 100) / 7 + POWER_Q;
                    targetHoodPos = shootTime > 700 ? 0.52 : 0.60;
                } else {
                    flyOutput = 0.55;
                    targetHoodPos = shootTime > 300 ? 0.5 : 0.54;
                }
                if (shootTime > 700) flyOutput += 0.04;
            } else {
                // Fallback di sicurezza se non vede il tag all'inizio: usa una potenza standard media
                flyOutput = 0.55;
                targetHoodPos = 0.54;
            }

            // Imposta velocità ai Flywheels
            double targetVelocity = flyOutput * 2500;
            ctx.topFly.setVelocity(targetVelocity);
            ctx.downFly.setVelocity(targetVelocity);

            // Aggiorna posizione servi dell'Hood
            ctx.rampL.setPosition(targetHoodPos);
            ctx.coverR.setPosition(1 - targetHoodPos);

            // 3. AUTOAIM DELLA TORRETTA (Dalla tua TeleOp)
            if (localQR[0] != -999) {
                double qrOffset = localQR[0] - (48 * Math.cos(Math.PI / 2 - Math.toRadians(localQR[3])) - CAMERA_OFFSET);
                double distance = localQR[1];
                double theta = Math.atan(qrOffset / distance);

                currentTurretPos = (Math.toDegrees(theta) * 2.63 + 165) / 300;
                currentTurretPos = Math.max(0.02, Math.min(0.98, currentTurretPos)); // Limiti di sicurezza
            }

            ctx.turretL.setPosition(currentTurretPos);
            ctx.turretR.setPosition(currentTurretPos);

            // 4. LOGICA DI SPARO AUTOMATICO (Simula la pressione intelligente di X)
            double currentVel = Math.abs(ctx.topFly.getVelocity());
            // Il Flywheel è pronto se gira ad almeno la velocità target meno una tolleranza di 150 tick/sec
            boolean flywheelReady = (targetVelocity > 100) && (currentVel >= (targetVelocity - 150));

            if (flywheelReady) {
                // Se i motori sono pronti e stiamo tracciando, apriamo il BallStop e azioniamo l'intake
                ctx.ballStop.setPosition(0);

                if (shootTime > 200) {
                    ctx.frontIn.setPower(0.4);
                    ctx.inRoller.setPower(1.0);
                } else {
                    ctx.frontIn.setPower(0);
                    ctx.inRoller.setPower(0);
                }
                shootTime += dt; // Incrementa il tempo di pressione virtuale di X
            } else {
                // Se i motori scendono di giri (es. quando passa la pallina), ferma momentaneamente l'intake per ricaricare
                ctx.frontIn.setPower(0);
                ctx.inRoller.setPower(0);
                ctx.ballStop.setPosition(0.25);
                // Non azzeriamo completamente shootTime se vuoi mantenere lo stadio dell'hood,
                // oppure puoi fare shootTime = 0 se preferisci resettare il ciclo ad ogni colpo.
            }

            // Telemetria di controllo per i test sul campo
            telemetry.addData("=== AUTO SHOOT ===", "ATTIVO");
            telemetry.addData("Tempo Mancante", "%.0f ms", durationMs - shootTimer.milliseconds());
            telemetry.addData("Distanza Target", localQR[1]);
            telemetry.addData("Flywheel Ready", flywheelReady);
            telemetry.addData("Vel Target", targetVelocity);
            telemetry.addData("Vel Attuale", currentVel);
            telemetry.update();

            idle();
        }

        // 5. SPEGNIMENTO DI SICUREZZA (Fine dell'azione)
        ctx.topFly.setVelocity(0);
        ctx.downFly.setVelocity(0);
        ctx.frontIn.setPower(0);
        ctx.inRoller.setPower(0);
        ctx.ballStop.setPosition(0.25);
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
    /*private boolean turretMovement(DcMotor turetta, double tRawPos, double minSpeed) {
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
    }*/

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
   /* private void turretHoming(TouchSensor toccami, DcMotor turetta) {

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
    }*/

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