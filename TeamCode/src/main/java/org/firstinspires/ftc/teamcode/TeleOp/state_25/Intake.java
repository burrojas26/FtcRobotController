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

    // Software limits for servos
    double pinchMax = 0; //TODO
    double pinchMin = 0; //TODO
    double rotateMax = 0; //TODO
    double rotateMin = 0; //TODO

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

    /**
     * Function to rotate the pinching servo based on two trigger inputs
     *
     * @param triggerPos The trigger value to use as clockwise rotation
     * @param triggerNeg The trigger value to use as counter clockwise rotation
     */
    public void pinch(float triggerPos, float triggerNeg) {
        double newPos = pinch.getPosition() + (triggerPos-triggerNeg)*0.005;
        if (newPos < pinchMax && newPos > pinchMin) {
            pinch.setPosition(newPos);
        }
    }

    /**
     * Function to rotate the rotation servo based on a joystick input
     *
     * @param joyStick The value from the joystick controlling rotation
     */
    public void rotate(double joyStick) {
        double newPos = rotate.getPosition() + joyStick*0.005;
        if (newPos < rotateMax && newPos > rotateMin) {
            rotate.setPosition(newPos);
        }
    }
}
