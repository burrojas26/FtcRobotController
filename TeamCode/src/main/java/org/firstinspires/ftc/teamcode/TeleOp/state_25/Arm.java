/**
 * @author Jasper Burroughs
 * This class represents an Arm mechanism for an FTC robot.
 * The arm consists of two motors for extension and two servos for rotation.
 * It includes methods for rotating, extending, collapsing, and manually controlling the arm.
 */

package org.firstinspires.ftc.teamcode.TeleOp.state_25;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    // Motors for arm extension
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;

    // Servos for arm rotation
    Servo leftServo;
    Servo rightServo;

    // Gamepad for manual control
    Gamepad gamepad2;

    // Limits for servo rotation and extension
    double servoMax = 0.72; // Maximum servo position
    double servoMin = 0.27; // Minimum servo position
    int extensionMax = 2625; // Maximum extension position
    int extSpeed = 4000; // Arm extension speed

    /**
     * Constructor to initialize the Arm with motors, servos, and a gamepad.
     *
     * @param leftMotor   The left extension motor
     * @param rightMotor  The right extension motor
     * @param leftServo   The left rotation servo
     * @param rightServo  The right rotation servo
     * @param gamepad     The gamepad for manual control
     */
    public Arm(DcMotorEx leftMotor, DcMotorEx rightMotor, Servo leftServo, Servo rightServo, Gamepad gamepad) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        this.gamepad2 = gamepad;
    }

    /**
     * Rotates the arm using the joystick input.
     * Adjusts the servo position based on joystick movement, within set limits.
     *
     * @param joyStick The joystick input value for rotation (-1 to 1).
     */
    public void rotateArm(double joyStick) {
        double newPos = leftServo.getPosition() + joyStick * 0.003;
        leftServo.setDirection(Servo.Direction.REVERSE); // Ensuring one servo moves in reverse
        if (newPos < servoMax && newPos > servoMin) { // Ensuring the position is within bounds
            leftServo.setPosition(newPos);
            rightServo.setPosition(newPos);
        }
    }

    public void armUp() {
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);
    }

    public void armDown() {
        leftServo.setPosition(servoMax);
        rightServo.setPosition(servoMax);
    }

    /**
     * Extends the arm to its maximum position.
     * Sets both motors to move to the pre-defined extension limit.
     */
    public void extendArm() {
        leftMotor.setTargetPosition(extensionMax);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setTargetPosition(extensionMax);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setVelocity(4000); // Set velocity for smooth movement
        rightMotor.setVelocity(4000);
    }

    /**
     * Retracts the arm back to its starting position (0).
     */
    public void collapseArm() {
        leftMotor.setTargetPosition(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setTargetPosition(0);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setVelocity(4000);
        rightMotor.setVelocity(4000);
    }

    /**
     * Stops the arm and holds position.
     */
    public void stopArm() {
        leftMotor.setTargetPosition(leftMotor.getCurrentPosition());
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition());
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setVelocity(500);
        rightMotor.setVelocity(500);
    }

    /**
     * Manually controls the arm extension based on gamepad input.
     * The right stick's Y-axis adjusts the motor position within safe limits.
     */
    public void manual() {
        float increase = -gamepad2.right_stick_y * 500; // Convert joystick movement to motor steps
        int newPos = leftMotor.getCurrentPosition() + (int) increase;
        if (newPos < extensionMax && newPos > 0) { // Ensure within limits
            leftMotor.setTargetPosition(newPos);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setTargetPosition(newPos);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setVelocity(extSpeed);
            rightMotor.setVelocity(extSpeed);
        }
    }

    /**
     * Resets the encoders for both motors, useful for recalibrating the arm.
     */
    public void resetEncode() {
        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

}
