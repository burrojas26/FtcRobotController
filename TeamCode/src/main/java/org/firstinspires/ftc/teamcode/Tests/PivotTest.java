/**
 * @author Jasper Burroughs
 * Test for the arm servos and motors
 */

package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Pivot Test", group="Linear OpMode")
@Disabled
public class PivotTest extends LinearOpMode {

    // Robot hardware variables
    CRServo leftServo, rightServo; // Arm rotation servos

    @Override
    public void runOpMode() {
        // Initialize arm motors and servos
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");

        waitForStart();

        // Declare control variables
        Gamepad oldGamepad1 = new Gamepad();
        oldGamepad1.copy(gamepad1);
        Gamepad oldGamepad2 = new Gamepad();
        oldGamepad2.copy(gamepad2);
        boolean oldBtn = false;
        double servoMax = 0.72; // Maximum servo position
        double servoMin = 0.27; // Minimum servo position

        leftServo.getController().setServoPosition(0, 0.5);
        rightServo.getController().setServoPosition(0, 0.5);

        while (opModeIsActive()) {
            double newPos = leftServo.getController().getServoPosition(0) + gamepad2.left_stick_y * 0.0005;
            leftServo.setDirection(CRServo.Direction.REVERSE); // Ensuring one servo moves in reverse
            if (newPos < servoMax && newPos > servoMin) { // Ensuring the position is within bounds
                leftServo.getController().setServoPosition(0, newPos);
                rightServo.getController().setServoPosition(0, newPos);
            }

            telemetry.addData("Left Servo", leftServo.getController().getServoPosition(0));
            telemetry.addData("Right Servo", rightServo.getController().getServoPosition(0));
            telemetry.update();

            // Update the old gamepad settings
            oldGamepad1.copy(gamepad1);
            oldGamepad2.copy(gamepad2);
        } // End while loop
    } // End runOpMode

}
