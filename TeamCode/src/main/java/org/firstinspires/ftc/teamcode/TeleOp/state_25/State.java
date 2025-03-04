/**
 * @author Jasper Burroughs
 * This is the final program for the teleop period in the 2024-25 state competition (Into the Deep).
 */

package org.firstinspires.ftc.teamcode.TeleOp.state_25;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;

@TeleOp(name="State Comp 2025", group="Linear OpMode")
public class State extends LinearOpMode {

    // Robot hardware variables
    Orientation angles;
    DcMotorEx leftFront, leftBack, rightFront, rightBack; // Drivetrain motors
    DcMotorEx leftArm, rightArm; // Arm motors
    Servo leftServo, rightServo; // Arm rotation servos
    Servo pinch, rotator; // Intake servos

    // Localization and Arm control
    GoBildaPinpointDriver pinpoint;
    Arm arm;
    Intake intake;

    // Control Variables
    double percent; // Power percentage for driving

    @Override
    public void runOpMode() {

        // Initialize drivetrain motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Initialize arm motors and servos
        leftArm = hardwareMap.get(DcMotorEx.class, "leftArm");
        rightArm = hardwareMap.get(DcMotorEx.class, "rightArm");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        // Initialize intake servos
        pinch = hardwareMap.get(Servo.class, "pinch");
        rotator = hardwareMap.get(Servo.class, "rotate");

        // Initialize GoBilda Pinpoint driver for localization
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Configure motor directions (ensuring forward movement aligns with joystick input)
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        // Configure Arm motor directions
        leftArm.setDirection(DcMotorSimple.Direction.FORWARD);
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set rotation of arm
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);

        // Configure the pinpoint localization system
        pinpoint.setOffsets(107.3, 0);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        // Reset encoders for the arm motors
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // Declare control variables
        percent = 65; // Default power percentage
        Gamepad oldGamepad1 = new Gamepad();
        Gamepad oldGamepad2 = new Gamepad();
        oldGamepad1.copy(gamepad1);
        oldGamepad2.copy(gamepad2);
        boolean manual = false; // Manual Mode for vertical slide
        boolean fieldCentric = false;


        while (opModeIsActive()) {
            // Update localization data
            pinpoint.update();

            // Retrieve joystick inputs for movement
            double strafe = gamepad1.left_stick_x;
            double drive = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;

            // Get the robot's current heading in radians
            double robotHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

            // Apply field-centric transformation using a rotation matrix
            double newX = strafe * Math.cos(robotHeading) + drive * Math.sin(robotHeading);
            double newY = -strafe * Math.sin(robotHeading) + drive * Math.cos(robotHeading);

            // Initialize Arm control with hardware inputs
            arm = new Arm(leftArm, rightArm, leftServo, rightServo, gamepad2);

            // Arm control logic
            if (gamepad2.right_stick_button && !oldGamepad2.right_stick_button) manual = !manual;
            if (manual) arm.manual();
            if (gamepad2.y) arm.extendArm();
            if (gamepad2.a) arm.collapseArm();
            if (gamepad2.x) arm.stopArm();
            if (gamepad2.right_bumper) arm.resetEncode();
            if (gamepad2.dpad_left) arm.armDown();
            if (gamepad2.dpad_up) arm.armUp();
            arm.rotateArm(gamepad2.left_stick_y);

            // Initialize Intake control
            intake = new Intake(pinch, rotator);

            // Intake control logic
            intake.pinch(gamepad2.right_trigger, gamepad2.left_trigger);
            if (!manual) intake.rotate(gamepad2.right_stick_y);

            // Emergency stop triggered by back button
            if (gamepad1.back || gamepad2.back) stopAll();

            // Gamepad2 can take over driving if left bumper is pressed
            if (gamepad2.left_bumper) {
                strafe = gamepad2.left_stick_x;
                drive = -gamepad2.left_stick_y;
                rotate = gamepad2.right_stick_x;
            }

            // Toggle between field-centric and robot-centric control
            if (gamepad1.ps && !oldGamepad1.ps) fieldCentric = !fieldCentric;

            // Adjust driving power percentage with d-pad
            if (percent < 100 && gamepad1.dpad_up && !oldGamepad1.dpad_up) percent += 5;
            if (percent > 0 && gamepad1.dpad_down && !oldGamepad1.dpad_down) percent -= 5;
            if (gamepad1.a) percent = 20; // Low-speed mode
            if (gamepad1.y) percent = 100; // Full speed
            if (gamepad1.b) percent = 65; // Default speed

            // Determine drive mode (field-centric or robot-centric)
            double x = fieldCentric ? newX : strafe;
            double y = fieldCentric ? newY : drive;

            // Execute driving function
            drive(x, y, rotate, percent);

            // Display telemetry data
            updateTelemetry(fieldCentric, manual);

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
        percent = 0;
        arm.stopArm();
    }

    /**
     * Controls the robot drive system using mecanum wheel calculations.
     *
     * @param drive   Forward/backward movement
     * @param strafe  Left/right movement
     * @param rotate  Rotational movement
     * @param percent Power percentage to scale movement
     */
    public void drive(double drive, double strafe, double rotate, double percent) {
        double scale = percent / 100;
        leftFront.setPower(scale * (drive + strafe + rotate));
        rightFront.setPower(scale * (-drive + strafe + rotate));
        leftBack.setPower(scale * (-drive + strafe - rotate));
        rightBack.setPower(scale * (drive + strafe - rotate));
    }

    /**
     * Updates all telemetry data
     *
     * @param fieldCentric boolean for field centric mode
     * @param manual boolean for manual control of the arm
     */
    public void updateTelemetry(boolean fieldCentric, boolean manual) {
        telemetry.addLine("Robot Data");
        telemetry.addData("Field Centric Mode", fieldCentric);
        telemetry.addData("Robot Position (X axis in CM)", pinpoint.getPosition().getX(DistanceUnit.CM));
        telemetry.addData("Robot Position (Y axis in CM)", pinpoint.getPosition().getY(DistanceUnit.CM));
        telemetry.addData("Robot Position (H axis in Degrees)", pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addLine("\nDrive Data");
        telemetry.addData("Percent Power", percent);
        telemetry.addData("Left Front Power", leftFront.getPower());
        telemetry.addData("Right Front Power", rightFront.getPower());
        telemetry.addData("Left Back Power", leftBack.getPower());
        telemetry.addData("Right Back Power", rightBack.getPower());
        telemetry.addLine("Arm Data");
        telemetry.addData("Manual", manual);
        telemetry.addData("Left Motor", leftArm.getCurrentPosition());
        telemetry.addData("Right Motor", rightArm.getCurrentPosition());
        telemetry.addData("Left Servo", leftServo.getPosition());
        telemetry.addData("Right Servo", rightServo.getPosition());
        telemetry.addLine("Intake Data");
        telemetry.addData("Pinch Position", pinch.getPosition());
        telemetry.addData("Rotate Position", rotator.getPosition());
        telemetry.update();
    }
}
