package org.firstinspires.ftc.teamcode.TeleOp.state_25;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Auto {
    // The classes for control
    Arm arm;
    Intake intake;

    //Enum for actions
    enum State {GET_FROM_WALL, READY_PICKUP, SCORE_SAMPLE, MANUAL}
    State currentState = State.MANUAL;

    ElapsedTime timer = new ElapsedTime();

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

    public void setState(State state) {
        currentState = state;
        timer.reset();
    }

    public void update() {
        switch (currentState) {
            case MANUAL:
                break;
            case GET_FROM_WALL:
                double armPos = 0.7361;
                double rotPos = 0.2072;
                double pinchPos = 0.1972;

                arm.rotateToPos(armPos);
                intake.rotateToPos(rotPos);
                intake.pinchToPos(pinchPos);


                break;
            case READY_PICKUP:
                double wristPos = 0.2633;
                double rotPos2 = 0.76;
                intake.rotateToPos(wristPos);
                intake.open();
                arm.rotateToPos(rotPos2);


                break;
            case SCORE_SAMPLE:
                int scoreExt = 1230;
                int scoreExt2 = 422;
                double scoreRotPos = 0.5889;
                double scoreWristPos = 0.2689;
                intake.rotateToPos(scoreWristPos);
                if (timer.milliseconds() > 500) {
                    arm.setArmPos(scoreExt);
                }
                if (timer.milliseconds() > 1100) {
                    arm.rotateToPos(scoreRotPos);
                }
                if (timer.milliseconds() > 1600) {
                    arm.setArmPos(scoreExt2);
                }

                break;
        }
    }

}
