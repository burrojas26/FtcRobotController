/**
 * @author Jasper Burroughs
 * This is the final program for the teleop period in the 2024-25 state competition (Into the Deep).
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

@TeleOp(name="State Comp 2025", group="Linear OpMode")
public class State extends LinearOpMode {

    // Robot hardware variables
    Orientation angles;
    DcMotorEx leftFront, leftBack, rightFront, rightBack; // Drivetrain motors
    DcMotorEx leftArm, rightArm; // Arm motors
    Servo leftServo, rightServo; // Arm rotation servos
    double percent; // Power percentage for driving

    // Localization and Arm control
    GoBildaPinpointDriver pinpoint;
    Arm arm;

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

        // Initialize GoBilda Pinpoint driver for localization
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Configure motor directions (ensuring forward movement aligns with joystick input)
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        // Configure the pinpoint localization system
        pinpoint.setOffsets(107.3, 0);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        waitForStart();

        // Declare control variables
        percent = 65; // Default power percentage
        Gamepad oldGamepad1 = gamepad1;
        Gamepad oldGamepad2 = gamepad2;
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

            // Initialize Arm control with gamepad2 inputs
            arm = new Arm(leftArm, rightArm, leftServo, rightServo, gamepad2);

            // Arm control logic
            if (gamepad2.right_stick_button && !oldGamepad2.right_stick_button) arm.manual();
            if (gamepad2.y) arm.extendArm();
            if (gamepad2.a) arm.collapseArm();
            if (gamepad2.x) arm.stopArm();
            if (gamepad2.right_bumper) arm.resetEncode();

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
        // Compute individual motor powers based on drive vectors
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = -drive + strafe + rotate;
        double backLeftPower = -drive + strafe - rotate;
        double backRightPower = drive + strafe - rotate;

        // Scale motor power by percentage unless overridden
        if (!gamepad1.left_bumper) {
            frontLeftPower *= percent / 100;
            frontRightPower *= percent / 100;
            backLeftPower *= percent / 100;
            backRightPower *= percent / 100;
        }

        // Set motor power values
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
    }
}
