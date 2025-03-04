package org.firstinspires.ftc.teamcode.TeleOp.state_25;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Auto {
    // The classes for control
    Arm arm;
    Intake intake;

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

    public void stopAll() {
        stopped = true;
    }

}
