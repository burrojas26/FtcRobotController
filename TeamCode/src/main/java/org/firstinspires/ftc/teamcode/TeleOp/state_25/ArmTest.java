/**
 * @author Jasper Burroughs
 * Test for the arm servos and motors
 */

package org.firstinspires.ftc.teamcode.TeleOp.state_25;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;

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
        Gamepad oldGamepad1 = gamepad1;
        Gamepad oldGamepad2 = gamepad2;
        boolean btn = false;
        boolean manual = false;

        leftServo.setPosition(0.68);
        rightServo.setPosition(0.68);

        while (opModeIsActive()) {
            // Initialize Arm control with gamepad2 inputs
            arm = new Arm(leftArm, rightArm, leftServo, rightServo, gamepad2);

            // Arm control logic
            if (gamepad2.right_stick_button && !btn) manual = !manual;
            btn = gamepad2.right_stick_button;
            if (manual) arm.manual();
            if (gamepad2.y) arm.extendArm();
            if (gamepad2.a) arm.collapseArm();
            if (gamepad2.x) arm.stopArm();
            if (gamepad2.right_bumper) arm.resetEncode();
            arm.rotateArm(gamepad2.left_stick_y);

            // Emergency stop triggered by back button
            if (gamepad1.back || gamepad2.back) stopAll();

            telemetry.addData("Manual", manual);
            telemetry.addData("Left Motor", leftArm.getCurrentPosition());
            telemetry.addData("Right Motor", rightArm.getCurrentPosition());
            telemetry.addData("Left Servo", leftServo.getPosition());
            telemetry.addData("Right Servo", rightServo.getPosition());
            telemetry.update();

            // Update the old gamepad settings
            oldGamepad1 = gamepad1;
            oldGamepad2 = gamepad2;
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
