package org.firstinspires.ftc.teamcode.Movmts;


import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardware;
import com.qualcomm.robotcore.hardware.Gamepad;

public class StandardMvmnt {
    private RobotHardware Robot;
    private Gamepad gamepad;

    double  SPEED = 0.5;

    double speed = SPEED;




    public StandardMvmnt(RobotHardware robot, Gamepad GamePad){

        this.Robot = robot;
        this.gamepad = GamePad;
    }

    public void move(){
        double lX = gamepad.left_stick_x;
        double lY = gamepad.left_stick_y;
        double rX = gamepad.right_stick_x;
        boolean left_bumper = gamepad.left_bumper;
        double left_trigger = gamepad.left_trigger;

        //gestione velocità movimento
        speedControl(left_bumper, left_trigger);

        // set joystick sinistro
        lX = Math.abs(lX) < .4 ? 0 : lX;
        lY = Math.abs(lY) < .4 ? 0 : lY;

        // set joystick destro
        rX = Math.abs(rX) < .2 ? 0 : rX;


        //definizione array motori e output movimento motori
        double[] MotArr = MotorOut(lX, lY, rX);
        Robot.lfD.setPower(MotArr[0] * speed);
        Robot.lbD.setPower(MotArr[1] * speed);
        Robot.rfD.setPower(MotArr[2] * speed);
        Robot.rbD.setPower(MotArr[3] * speed);
    }

    //--------------------------------------------
    //       CONTROLLO VELOCITA DI MOVIMENTO
    //--------------------------------------------
    private void speedControl(boolean left_bumper, double left_trigger){
        if (left_bumper) {
            speed = 1; // l1  --> velocità massima --> potenza 1
        } else if (left_trigger >= 0.1) {
            speed = (SPEED - 0.1) * (1 - left_trigger) + 0.1; // l2 --> frenata
        } else {
            speed = SPEED; // velocità standard di sistema
        }
    }



    //--------------------------------------------
    //       CONTROLLO MOTORI MOVIMENTO
    //--------------------------------------------
    private double[] MotorOut(double lX, double lY, double rX) {
        double rot = rX;
        double y = lY, x = lX;
        double lf = y + x + rot;
        double lb = y - x + rot;
        double rf = y - x - rot;
        double rb = y + x - rot;
        double max = Math.max(1, Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb)))));
        return new double[]{lf / max, lb / max, rf / max, rb / max};
    }

}

