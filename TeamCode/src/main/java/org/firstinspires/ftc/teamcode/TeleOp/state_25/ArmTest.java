/**
 * @author Jasper Burroughs
 * Test for the arm servos and motors
 */

package org.firstinspires.ftc.teamcode.TeleOp.state_25;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Arm Test", group="Linear OpMode")
public class ArmTest extends LinearOpMode {

    // Robot hardware variables
    DcMotorEx leftArm, rightArm; // Arm motors
    Servo leftServo, rightServo; // Arm rotation servos

    Arm arm;

    @Override
    public void runOpMode() {
        // Initialize arm motors and servos
        leftArm = hardwareMap.get(DcMotorEx.class, "leftArm");
        rightArm = hardwareMap.get(DcMotorEx.class, "rightArm");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        waitForStart();

        // Declare control variables
        Gamepad oldGamepad1 = new Gamepad();
        oldGamepad1.copy(gamepad1);
        Gamepad oldGamepad2 = new Gamepad();
        oldGamepad2.copy(gamepad2);
        boolean manual = false;
        boolean oldBtn = false;


        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftServo.setPosition(0.5);
        leftServo.setPosition(0.5);

        while (opModeIsActive()) {
            // Initialize Arm control with gamepad2 inputs
            arm = new Arm(leftArm, rightArm, leftServo, rightServo, gamepad2);

            // Arm control logic
            if (gamepad2.right_stick_button && oldBtn) manual = !manual;
            oldBtn = gamepad2.right_stick_button;
            if (manual) arm.manual();
            if (gamepad2.y) arm.extendArm();
            if (gamepad2.a) arm.collapseArm();
            if (gamepad2.x) arm.stopArm();
            if (gamepad2.right_bumper) arm.resetEncode();
            if (gamepad2.dpad_left) arm.armDown();
            if (gamepad2.dpad_up) arm.armUp();
            arm.rotateArm(gamepad2.left_stick_y);

            // Emergency stop triggered by back button
            if (gamepad1.back || gamepad2.back) stopAll();

            telemetry.addData("Manual", manual);
            telemetry.addData("Left Arm", leftArm.getCurrentPosition());
            telemetry.addData("Right Arm", rightArm.getCurrentPosition());
            telemetry.addData("Left Motor", leftArm.getCurrentPosition());
            telemetry.addData("Right Motor", rightArm.getCurrentPosition());
            telemetry.addData("Left Servo", leftServo.getPosition());
            telemetry.addData("Right Servo", rightServo.getPosition());
            telemetry.update();

            // Update the old gamepad settings
            oldGamepad1.copy(gamepad1);
            oldGamepad2.copy(gamepad2);
        } // End while loop
    } // End runOpMode

    /**
     * Stops all robot movement.
     * Sets power to zero and stops the arm.
     */
    public void stopAll() {
        arm.stopArm();
    }
}
