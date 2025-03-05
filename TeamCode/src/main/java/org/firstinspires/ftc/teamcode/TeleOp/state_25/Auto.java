package org.firstinspires.ftc.teamcode.TeleOp.state_25;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Auto {
    // The classes for control
    Arm arm;
    Intake intake;

    scoreFn runnable = new scoreFn();
    Thread thread = new Thread(runnable);

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

    public void getFromWall() {
        double armPos = 0; //TODO
        int extPos = 0; //TODO
        double rotPos = 0; //TODO
        while (!stopped) {
            arm.rotateToPos(armPos);
            arm.setArmPos(extPos);
            intake.rotateToPos(rotPos);
            intake.open();
        }
    }

    public void readyPickup() {
        intake.rotateToPos(intake.rotateMax);
        intake.open();
    }

    class scoreFn implements Runnable {
        @Override
        public void run() {
                int scoreExt = 1300; //TODO
                int scoreExt2 = 800; //TODO
                double scoreRotPos = 0.7; //TODO
                double scoreRotPos2 = 0.65; //TODO
                double scoreWristPos = 0; //TODO
                intake.rotateToPos(scoreWristPos);
                pause(500);
                arm.rotateToPos(scoreRotPos);
                pause(500);
                arm.setArmPos(scoreExt);
                pause(600);
                arm.rotateToPos(scoreRotPos2);
                pause(500);
                arm.setArmPos(scoreExt2);
        }
    }

    public void scoreSample() {
        thread.start();
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
