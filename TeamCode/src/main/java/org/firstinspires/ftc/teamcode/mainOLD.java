package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class mainOLD extends LinearOpMode {

    int oRP, oLP, oAP;
    double oT, vClawTime, oClawTime, vClawTime2;
    boolean bTranslating = false, vClawCheck = false, vClawCheck2 = false, oClawCheck = false;

    //173,5 180,5
    double L = 17.7, O = 8.9, R = 2, N = 8192, correctionFact = 3f/180*Math.PI;//0.4;
    double cm = 2 * Math.PI * R / N, a = Math.atan2(O, L), P = Math.hypot(O, L);
    double[] pos = {0, 0, 0};

    @Override
    public void runOpMode() {
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        DcMotor leftVerticalSlider = hardwareMap.get(DcMotor.class, "lvs");
        DcMotor rightVerticalSlider = hardwareMap.get(DcMotor.class, "rvs");
        DcMotor leftOrizontalSlider = hardwareMap.get(DcMotor.class, "los");
        DcMotor rightOrizontalSlider = hardwareMap.get(DcMotor.class, "ros");
        Servo vClawS = hardwareMap.get(Servo.class, "vcs");
        Servo vHandS = hardwareMap.get(Servo.class, "vhs");
        Servo vArmS = hardwareMap.get(Servo.class, "vas");
        Servo oClawS = hardwareMap.get(Servo.class, "ocs");
        Servo oHandS = hardwareMap.get(Servo.class, "ohs");
        Servo oArmS = hardwareMap.get(Servo.class, "oas");

        DcMotor encoder1 = hardwareMap.get(DcMotor.class, "lb");
        DcMotor encoder2 = hardwareMap.get(DcMotor.class, "rb");
        DcMotor encoder3 = hardwareMap.get(DcMotor.class, "lf");

        encoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftVerticalSlider.setDirection(DcMotor.Direction.FORWARD);
        rightVerticalSlider.setDirection(DcMotor.Direction.FORWARD);
        leftOrizontalSlider.setDirection(DcMotor.Direction.FORWARD);
        rightOrizontalSlider.setDirection(DcMotor.Direction.FORWARD);

        leftVerticalSlider.setTargetPosition(0);
        rightVerticalSlider.setTargetPosition(0);
        leftOrizontalSlider.setTargetPosition(0);
        rightOrizontalSlider.setTargetPosition(0);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftVerticalSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerticalSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOrizontalSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOrizontalSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVerticalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVerticalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftOrizontalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOrizontalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVerticalSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVerticalSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftVerticalSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVerticalSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftOrizontalSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightOrizontalSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ctx ctx = new ctx(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, encoder1, encoder2, encoder3);

        waitForStart();

        double fact = .65; //dumping

        while (opModeIsActive()) {
            odometry(ctx);
            double x = pos[0], y = pos[1], t = pos[2];

            if (!leftOrizontalSlider.isBusy() && !rightOrizontalSlider.isBusy()) {
                leftOrizontalSlider.setPower(0);
                rightOrizontalSlider.setPower(0);
            } else {
                leftOrizontalSlider.setPower(1);
                rightOrizontalSlider.setPower(1);
            }

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("t", t/Math.PI*180);

            double lX = gamepad1.left_stick_x, lY = -gamepad1.left_stick_y;
            lX = Math.abs(lX) < .4 ? 0 : lX; //dead zone
            lY = Math.abs(lY) < .4 ? 0 : lY;
            double rX = gamepad1.right_stick_x, rY = -gamepad1.right_stick_y;
            rX = Math.abs(rX) < .2 ? 0 : rX;
            rY = Math.abs(rY) < .2 ? 0 : rY;

            if (!bTranslating && ( lX != 0 && lY != 0)) {
                bTranslating = true;
            } else if (bTranslating && ( lX == 0 && lY == 0)) {
                bTranslating = false;
            }
            if (!bTranslating) {
                oT = t;
            }

            //double[] fCtr = processRawCtr(lX, lY);

            double[] MotArr = MotorOut(-lX, -lY/*fCtr[0], fCtr[1]*/, rX, rY);

            telemetry.addData("mariio", "%.2f %.2f %.2f %.2f", MotArr[0], MotArr[1], MotArr[2], MotArr[3]);

            ctx.lFd.setPower(MotArr[0] * fact);
            ctx.lBd.setPower(MotArr[1] * fact);
            ctx.rFd.setPower(MotArr[2] * fact);
            ctx.rBd.setPower(MotArr[3] * fact);

            if (gamepad2.triangle) {
                oHandS.scaleRange(-1, 1);
                leftOrizontalSlider.setTargetPosition(-60);
                rightOrizontalSlider.setTargetPosition(60);
                if (gamepad2.dpad_down) {
                    if (oClawCheck) {
                        oClawCheck = false;
                        oArmS.setPosition(0.45);
                        oClawTime = getRuntime();
                    }
                    if (getRuntime() - oClawTime > .3) {
                        oClawS.setPosition(0.3);
                    }
                } else {
                    oClawCheck = true;
                    oArmS.setPosition(0.4);
                    oClawS.setPosition(0);
                }
                if (gamepad2.left_bumper) {
                    oHandS.setPosition(oHandS.getPosition() - 0.02);
                } else if (gamepad2.right_bumper) {
                    oHandS.setPosition(oHandS.getPosition() + 0.02);
                }
            } else {
                leftOrizontalSlider.setTargetPosition(0);
                rightOrizontalSlider.setTargetPosition(0);
                oArmS.setPosition(0.08);
                oHandS.setPosition(-.4);
            }

            if (gamepad2.circle) {
                if (vClawCheck) {
                    vClawCheck = false;
                    vClawTime = getRuntime();
                    vArmS.setPosition(0.6);
                }
                if (getRuntime() - vClawTime > 0.7) {
                    vClawS.setPosition(1);
                }
                if (getRuntime() - vClawTime > 1.5) {
                    oClawS.setPosition(0);
                }
                if (getRuntime() - vClawTime > 2) {
                    leftVerticalSlider.setPower(1);
                    rightVerticalSlider.setPower(1);
                    if (gamepad2.right_trigger > 0.1) {
                        vHandS.setPosition(.2);

                        leftVerticalSlider.setTargetPosition(3800);
                        rightVerticalSlider.setTargetPosition(3800);
                        if (gamepad2.dpad_up) {
                            vArmS.setPosition(.4);
                        } else {
                            vArmS.setPosition(0.24);
                        }
                    } else {
                        vHandS.setPosition(0);
                        vArmS.setPosition(0.05);
                        leftVerticalSlider.setTargetPosition(0);
                        rightVerticalSlider.setTargetPosition(0);
                    }
                }
                if (getRuntime() - vClawTime > 4 && gamepad2.left_trigger >= 0.1) {
                    vClawS.setPosition(0.6);
                }

            } else if (gamepad2.square) {
                if (vClawCheck2) {
                    vClawCheck2 = false;
                    vClawTime2 = getRuntime();
                    vArmS.setPosition(0.06);
                    vHandS.setPosition(.3);
                }
                if (gamepad2.left_trigger >= 0.1) {
                    vClawS.setPosition(.6);
                } else {
                    vClawS.setPosition(1);
                }
                if (gamepad2.cross) {
                    vHandS.setPosition(.4);
                    vArmS.setPosition(0.2);
                    leftVerticalSlider.setPower(1);
                    rightVerticalSlider.setPower(1);
                    leftVerticalSlider.setTargetPosition(1000);
                    rightVerticalSlider.setTargetPosition(1000);
                }
            } else {
                vClawCheck = true;
                vClawCheck2 = true;
                vHandS.setPosition(0.4);
                vArmS.setPosition(0.49);
                vClawS.setPosition(0.6);
                leftVerticalSlider.setPower(1);
                rightVerticalSlider.setPower(1);
                leftVerticalSlider.setTargetPosition(0);
                rightVerticalSlider.setTargetPosition(0);
                if (!leftVerticalSlider.isBusy() && !rightVerticalSlider.isBusy()) {
                    leftVerticalSlider.setPower(0);
                    rightVerticalSlider.setPower(0);
                }
            }

            if (gamepad1.left_bumper) {
                fact = 0.4;
            } else if (gamepad1.left_trigger >= 0.1){
                fact = 1;
            } else {
                fact = 0.7;
            }

            telemetry.update();
        }
    }

    private double[] MotorOut(double lX, double lY, double rX, double rY) {
        if (rX == 0) {
            double a = Math.atan2(lY, lX), p = Math.hypot(lX, lY);

            double sin = Math.sin(a - Math.PI / 4), cos = Math.cos(a - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double o1 = p * cos / max, o2 = p * sin / max;

            double[] c = correction();

            //lf, lb, rf, rb
            return new double[]{o1*c[0], o2*c[0], o2 *c[1], o1*c[1]};
        } else {
            //Rotazione ->
            return new double[]{rX, rX, -rX, -rX};
        }
    }

    private double getAng(double a, int dir) {
        return 0.5 + 1f/135f*a*dir; //1dx -1sx(il lato brutto)
    }

    private void odometry(ctx ctx) {
        int dR = -(ctx.e2.getCurrentPosition() - oRP), dL = ctx.e1.getCurrentPosition() - oLP, dA = ctx.e3.getCurrentPosition() - oAP;
        double x = pos[0], y = pos[1], t = pos[2];

        double dC = ((double)(dL-dR)*(Math.cos(a)))*cm*2*Math.PI;
        double dt = dC/(2f*Math.PI*P);
        t+=dt;

        double dx = ((double) (dL + dR)/2f*Math.sin(t)+dA*Math.cos(t))*cm;
        double dy = ((double) (dL + dR)/2f*Math.cos(t)+dA*Math.sin(t))*cm;
        x+=dx*2f;
        y+=dy*2f;

        oRP = ctx.e2.getCurrentPosition(); oLP = ctx.e1.getCurrentPosition(); oAP = ctx.e3.getCurrentPosition();
        pos[0]=x; pos[1] = y; pos[2] = t;
    }

    private double[] processRawCtr(double lX, double lY) {
        double ctrAngle = Math.atan2(lY, lX);
        telemetry.addData("cA", ctrAngle);
        double fnlAngle = ctrAngle + pos[2];
        telemetry.addData("fA", fnlAngle);
        double c = Math.hypot(lX, lY);
        return new double[]{c * Math.cos(fnlAngle), c * Math.sin(fnlAngle)};
    }

    private double[] correction() {
        double dt = pos[2] - oT;

        double c = 1-0.1*dt/correctionFact;

        if (dt < 0) {
            return new double[] {1, c};
        } else {
            return new double[] {c, 1};
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