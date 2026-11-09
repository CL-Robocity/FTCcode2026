package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import com.qualcomm.robotcore.hardware.HardwareMap;
public class RobotHardware{
    //costruttore di classe
    public RobotHardware(){}

    private int KP_FACTOR = 6;
    //variabile di imu
    public IMU imu;

    // variabile delle odo
    public DcMotor odoParallel;
    public DcMotor odoPerp;

    // variabile motori di movimento
    public DcMotor lfD; // sinistro avanti
    public DcMotor lbD; // sinistro dietro
    public DcMotor rfD; // destro avanti
    public DcMotor rbD; // destro dietro

    // motori flywheel
    public DcMotorEx TopFlyWheel;
    public DcMotorEx DownFlyWheel;

    // PIDF COEFFICIENTS
    private PIDFCoefficients currentPIDF;
    private PIDFCoefficients aggressivePIDF;

    // variabile motori intake
    public DcMotor FrontIntake; // motore intake
    public DcMotor IntakeRoller;

    // variabili servo
    public Servo LeftTurretServo;
    public Servo RightTurretServo;
    public Servo BallStopServo;
    public Servo RampLeftServo;
    public Servo CoverRightServo;
    public GoBildaPinpointDriver pinpoint;



    public void init(HardwareMap hwmp, Telemetry telemetry) throws InterruptedException{

/*
        imu = hwmp.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.DOWN
            )
        );
        imu.initialize(parameters);
        Thread.sleep(1000);
        imu.resetYaw();
        // ================= PINPOINT =================
*/
        pinpoint = hwmp.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

// Configurazione dead wheels goBILDA 4-Bar
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

// Offset rispetto al centro geometrico del robot
        pinpoint.setOffsets(
                -102.5,
                -57.5,
                DistanceUnit.MM
        );

// Direzione iniziale degli encoder
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

// Reset posizione e calibrazione IMU interna
// Il robot deve rimanere fermo durante questa operazione
        pinpoint.resetPosAndIMU();

// aspetta la fine della calibrazione IMU (robot fermo!)
        Thread.sleep(300);
        long t0 = System.currentTimeMillis();
        while (System.currentTimeMillis() - t0 < 3000) {
            pinpoint.update();
            if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) break;
            Thread.sleep(10);
        }

        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());

        telemetry.addData("Pinpoint", "Configured");

/*

        // set della odo
        odoParallel = hwmp.get(DcMotor.class, "roller");
        odoPerp = hwmp.get(DcMotor.class, "frontintake");


        //reset encoder delle odo
        odoParallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoParallel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


*/

        // riconoscimento motori di movimento
        lfD = hwmp.get(DcMotor.class, "lf");
        lbD = hwmp.get(DcMotor.class, "lb");
        rfD = hwmp.get(DcMotor.class, "rf");
        rbD = hwmp.get(DcMotor.class, "rb");


        // set direzione motori
        lfD.setDirection(DcMotor.Direction.REVERSE);
        lbD.setDirection(DcMotor.Direction.REVERSE);
        rfD.setDirection(DcMotor.Direction.FORWARD);
        rbD.setDirection(DcMotor.Direction.FORWARD);

        // set per runnare i motori
        lfD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set movimento a potenza 0
        lfD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


/*
        // set motori d'azione
        TopFlyWheel = (DcMotorEx) hwmp.get(DcMotor.class, "frontintake"); // motore flywheel
        DownFlyWheel = (DcMotorEx) hwmp.get(DcMotor.class, "roller"); // motore flywheel pt 2

        // reset encoder flywheel
        TopFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DownFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TopFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DownFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // set motori flywheel con PIDF
        currentPIDF = TopFlyWheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        aggressivePIDF = new PIDFCoefficients(currentPIDF.p * KP_FACTOR, currentPIDF.i, currentPIDF.d, currentPIDF.f);

        TopFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, aggressivePIDF);
        DownFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, aggressivePIDF);




       /* FrontIntake = hwmp.get(DcMotor.class, "frontintake"); // motore intake
        IntakeRoller = hwmp.get(DcMotor.class, "roller"); // motore roller

        FrontIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Motors: ", "Ready :)");


        // set servo
        LeftTurretServo = hwmp.get(Servo.class, "turretleft");
        RightTurretServo = hwmp.get(Servo.class, "turretright");
        BallStopServo = hwmp.get(Servo.class, "ballstop");
        RampLeftServo = hwmp.get(Servo.class, "rampleft");
        CoverRightServo = hwmp.get(Servo.class, "coverright");*/


        telemetry.addData("Servo: ", "Ready :)");
    }


}