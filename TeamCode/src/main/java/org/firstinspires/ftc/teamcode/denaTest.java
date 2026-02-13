

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="Auto Main: Guida a Tempo", group="Main")
public class denaTest extends LinearOpMode {

    private DcMotor lfD = null;
    private DcMotor lbD = null;
    private DcMotor rfD = null;
    private DcMotor rbD = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Definisci le velocità
    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        // Inizializza le variabili hardware usando i nomi del file 'main'
        lfD = hardwareMap.get(DcMotor.class, "lf");
        lbD = hardwareMap.get(DcMotor.class, "lb");
        rfD = hardwareMap.get(DcMotor.class, "rf");
        rbD = hardwareMap.get(DcMotor.class, "rb");

        // Imposta le direzioni dei motori come nel file 'main'
        // I motori di sinistra sono REVERSE, quelli di destra FORWARD
        lfD.setDirection(DcMotor.Direction.REVERSE);
        lbD.setDirection(DcMotor.Direction.REVERSE);
        rfD.setDirection(DcMotor.Direction.FORWARD);
        rbD.setDirection(DcMotor.Direction.FORWARD);

        // Imposta il comportamento 'BRAKE' come nel file 'main'
        lfD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Invia un messaggio di telemetria per segnalare che il robot è in attesa
        telemetry.addData("Status", "Pronto a partire");
        telemetry.update();

        // Attendi l'inizio della partita (il pilota preme START)
        waitForStart();

        // Esegui ogni fase del percorso




        // --- INIZIO CODICE GENERATO ---

        // Comando 1: wheels(1,1,1,1,3)
        lfD.setPower(0.2);
        rfD.setPower(0.2);
        lbD.setPower(0.2);
        rbD.setPower(0.2);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <3.000000 )){
            telemetry.addData("Percorso", "Fase 1 (Avanti): %4.1f S trascorsi", runtime.seconds());
            telemetry.update();
        }        // --- FINE CODICE GENERATO ---
        lfD.setPower(0);
        rfD.setPower(0);
        lbD.setPower(0);
        rbD.setPower(0);
    }
}
