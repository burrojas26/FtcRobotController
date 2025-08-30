/**
 * @author Jasper Burroughs
 * This class represents an intake mechanism for an FTC robot.
 * The intake consists of two servos for rotation and pinching.
 * It includes methods for rotating and pinching.
 */

package org.firstinspires.ftc.teamcode.TeleOp.state_25;

import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    // Servos for intake control
    Servo pinch, rotate;

    Auto autoClass;

    // Software limits for servos
    double pinchMax = 0.3;
    double pinchMin = 0;
    double rotateMax = 0.28;
    double rotateMin = 0.17;

    /**
     * Constructor to initialize the intake with 2 servos
     *
     * @param pinch Servo that controls pinching motion
     * @param rotate Servo that controls rotation
     */
    public Intake(Servo pinch, Servo rotate) {
        this.pinch = pinch;
        this.rotate = rotate;
    }

    public void getAuto(Auto autoClass) {
        this.autoClass = autoClass;
    }

    public void overrideAuto() {
        autoClass.setState(Auto.State.MANUAL);
        autoClass.update();
    }

    /**
     * Function to rotate the pinching servo based on two trigger inputs
     *
     * @param triggerPos The trigger value to use as clockwise rotation
     * @param triggerNeg The trigger value to use as counter clockwise rotation
     */
    public void pinch(float triggerPos, float triggerNeg) {
        overrideAuto();
        double newPos = pinch.getPosition() + (triggerPos-triggerNeg)*0.008;
        pinchToPos(newPos);
    }

    public void pinchToPos(double newPos) {
        if (newPos <= pinchMax && newPos >= pinchMin) {
            overrideAuto();
            pinch.setPosition(newPos);
        }
    }

    public void open() {
        overrideAuto();
        pinchToPos(pinchMax);
    }

    public void close() {
        double closePos = 0;
        overrideAuto();
        pinchToPos(closePos);
    }

    /**
     * Function to rotate the rotation servo based on a joystick input
     *
     * @param joyStick The value from the joystick controlling rotation
     */
    public void rotate(double joyStick) {
        overrideAuto();
        double newPos = rotate.getPosition() + joyStick*0.005;
        rotateToPos(newPos);
    }

    public void rotateToPos(double newPos) {
        if (newPos < rotateMax && newPos > rotateMin) {
            overrideAuto();
            rotate.setPosition(newPos);
        }
    }

    /**
     * Function to stop all moving parts
     */
    public void stopIntake() {
        overrideAuto();
        pinch.setPosition(pinch.getPosition());
        rotate.setPosition(rotate.getPosition());
    }
}
