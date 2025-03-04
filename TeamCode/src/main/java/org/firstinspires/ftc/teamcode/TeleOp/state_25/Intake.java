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
    double pinchMax = 0.3;
    double pinchMin = 0;
    double rotateMax = 0.5;
    double rotateMin = 0.4;

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
        double newPos = pinch.getPosition() + (triggerPos-triggerNeg)*0.008;
        pinchToPos(newPos);
    }

    public void pinchToPos(double newPos) {
        if (newPos < pinchMax && newPos > pinchMin) {
            pinch.setPosition(newPos);
        }
    }

    public void open() {
        pinchToPos(pinchMax);
    }

    public void close() {
        double closePos = 0; //TODO
        pinchToPos(closePos);
    }

    /**
     * Function to rotate the rotation servo based on a joystick input
     *
     * @param joyStick The value from the joystick controlling rotation
     */
    public void rotate(double joyStick) {
        double newPos = rotate.getPosition() + joyStick*0.005;
        rotateToPos(newPos);
    }

    public void rotateToPos(double newPos) {
        if (newPos < rotateMax && newPos > rotateMin) {
            rotate.setPosition(newPos);
        }
    }

    /**
     * Function to stop all moving parts
     */
    public void stopIntake() {
        pinch.setPosition(pinch.getPosition());
        rotate.setPosition(rotate.getPosition());
    }
}
