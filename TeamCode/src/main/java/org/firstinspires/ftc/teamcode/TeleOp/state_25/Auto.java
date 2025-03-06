package org.firstinspires.ftc.teamcode.TeleOp.state_25;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Auto {
    // The classes for control
    Arm arm;
    Intake intake;

    scoreFn scorer = new scoreFn();
    Thread scoreThread = new Thread(scorer);

    readyFn ready = new readyFn();
    Thread readyThread = new Thread(ready);

    fromWallFn fromWall = new fromWallFn();
    Thread wallThread = new Thread(fromWall);

    boolean stopped = false;

    /**
     * Constructor to initialize the Arm with motors, servos, and a gamepad.
     *
     * @param arm           The class controlling the arm
     * @param intake        The class controlling the intake
     */
    public Auto(Arm arm, Intake intake) {
        this.arm = arm;
        this.intake = intake;
    }

    class fromWallFn implements Runnable {
        @Override
        public void run() {
            double armPos = 0.7361;
            double rotPos = 0.2072;
            double pinchPos = 0.1972;
            arm.rotateToPos(armPos);
            intake.rotateToPos(rotPos);
            intake.pinchToPos(pinchPos);
        }
    }

    public void getFromWall() {
        wallThread.start();
    }

    class readyFn implements Runnable {
        @Override
        public void run() {
            double wristPos = 0.2633;
            double rotPos = 0.76;
            intake.rotateToPos(wristPos);
            intake.open();
            arm.rotateToPos(rotPos);
        }
    }

    public void readyPickup() {
        readyThread.start();
    }

    class scoreFn implements Runnable {
        @Override
        public void run() {
                int scoreExt = 1230;
                int scoreExt2 = 422;
                double scoreRotPos = 0.5889;
                double scoreWristPos = 0.2689;
                intake.rotateToPos(scoreWristPos);
                pause(500);
                arm.setArmPos(scoreExt);
                pause(600);
                arm.rotateToPos(scoreRotPos);
                pause(500);
                arm.setArmPos(scoreExt2);
        }
    }

    public void scoreSample() {
        scoreThread.start();
    }

    public void pause(long time) {
        try {
            // Sleep for 2 seconds (2000 milliseconds)
            Thread.sleep(time);
        } catch (InterruptedException e) {
            System.out.println("Thread was interrupted!");
        }
    }

    public void stopAll() {
        stopped = true;
    }

}
