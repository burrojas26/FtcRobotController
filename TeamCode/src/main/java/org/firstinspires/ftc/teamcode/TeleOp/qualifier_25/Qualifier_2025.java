/**
 * @author Jasper Burroughs
 * This is the final program for the teleop period in the 2024-25 competition (Into the deep)
 * This was the code for our qualifier. The robot and code have been redesigned for state
 */


package org.firstinspires.ftc.teamcode.TeleOp.qualifier_25;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp
@Disabled
public class Qualifier_2025 extends LinearOpMode {
    //Variables
    IMU imu;
    Orientation angles;
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    DcMotorEx arm;
    Servo hand;
    Servo inputServo;
    DcMotorEx intakeMotor;
    DcMotorEx hSlide;

    double percent;
    HorizontalMode horizontalMode;
    VerticalMode verticalMode;

    @Override

    public void runOpMode() {
        // Get the IMU instance
        imu = hardwareMap.get(IMU.class, "imu");
        // Initializing hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        arm = hardwareMap.get(DcMotorEx.class, "slide");
        hand = hardwareMap.get(Servo.class, "hand");
        inputServo = hardwareMap.get(Servo.class, "inputServo");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        hSlide = hardwareMap.get(DcMotorEx.class, "hSlide");

        horizontalMode = new HorizontalMode(inputServo, hand, intakeMotor, hSlide, gamepad2);
        verticalMode = new VerticalMode(arm, hand, gamepad2);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientation));

        // All motors facing forward for the most recent build of the robot (go builda kit)
        // Some chassis builds require reversal of two of the four motors
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        // Setting the position of the arm to 0 at initialization
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Setting the position of the intakeArm to 0 at initialization
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        hSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        /* Declaring variables
        Percent is the percentage of power for driving
        PastGP is a gamepad that is reset at every run of the while loop
        */
        percent = 65;
        boolean dpadUp = false;
        boolean dpadDown = false;
        boolean fieldCentric = false;
        boolean bBtn = false;
        boolean rtStickBtn = false;
        boolean vertical = true;
        boolean manual = false;
        boolean ps = false;
        while (opModeIsActive()) {
            // Getting inputs for driving
            double strafe = gamepad1.left_stick_x;
            double drive = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;


            // Get the robot's current heading in radians
            // Retrieve the robot's orientation
            Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double robotHeading = Math.toRadians(angles.firstAngle);

            // Field-centric transformation using Rotation Matrix
            double newX = strafe * Math.cos(robotHeading) + drive * Math.sin(robotHeading);
            double newY = -strafe * Math.sin(robotHeading) + drive * Math.cos(robotHeading);

            // Automatically stops everything
            if (gamepad1.back || gamepad2.back) stopAll();

            // Allows gamepad2 to take over driving
            if (gamepad2.left_bumper) {
                strafe = gamepad2.left_stick_x;
                drive = -gamepad2.left_stick_y;
                rotate = gamepad2.right_stick_x;
            }

            // Toggles between field centric and robot centric
            if (gamepad1.ps && !ps) {
                fieldCentric = !fieldCentric;
            }
            ps = gamepad1.ps;
            // Switch the mode to horizontal arm
            if (gamepad2.b && !bBtn) {
                vertical = !vertical;
                manual = false;
            }
            bBtn = gamepad2.b;

            // Switch the arm mode to automatic
            if (gamepad2.right_stick_button && !rtStickBtn) {
                manual = !manual;
            }
            rtStickBtn = gamepad2.right_stick_button;

            // Code for when controller is in horizontal mode
            if (!vertical) {

                // Active intake servo and pivot code
                // Spin the servo when the triggers are pressed
                if (gamepad2.right_trigger != 0 && (inputServo.getController().getServoPosition(inputServo.getPortNumber()) + 0.05) <= 1) {
                    inputServo.getController().setServoPosition(inputServo.getPortNumber(), inputServo.getController().getServoPosition(inputServo.getPortNumber()) + 0.005);
                } else if (gamepad2.left_trigger != 0 && (inputServo.getController().getServoPosition(inputServo.getPortNumber()) - 0.05) >= 0) {
                    inputServo.getController().setServoPosition(inputServo.getPortNumber(), inputServo.getController().getServoPosition(inputServo.getPortNumber()) - 0.005);
                } else {
                    inputServo.getController().setServoPosition(inputServo.getPortNumber(), inputServo.getController().getServoPosition(inputServo.getPortNumber()));
                }

                // Intake Automation Code
                if (gamepad2.dpad_up) horizontalMode.activateIntake();
                // Control horizontal arm pivot
                if (!gamepad2.left_bumper) horizontalMode.intakeManual();
                // Reset encoder for horizontal arm pivot
                if (gamepad2.dpad_left) horizontalMode.resetIntake();

                // Horizontal Arm extension code
                // Pre-programmed instructions and manual control
                if (!manual) {
                    if (gamepad2.x) horizontalMode.stopExt();
                    if (gamepad2.right_bumper) horizontalMode.resetExt();
                    if (gamepad2.y) horizontalMode.extendArm();
                    if (gamepad2.a) horizontalMode.collapseArm();
                } else if (!gamepad2.left_bumper) horizontalMode.manual();
            }

            // Controls for when the controller is in vertical mode
            if (vertical) {
                // Vertical Arm extension control
                if (!manual) {
                    if (gamepad2.x) verticalMode.stopArm();
                    if (gamepad2.right_bumper) verticalMode.resetEncode();
                    if (gamepad2.y) verticalMode.extendArm();
                    if (gamepad2.a) verticalMode.collapseArm();
                    if (gamepad2.ps) verticalMode.hang();
                } else if (!gamepad2.left_bumper) verticalMode.manual();

                // This is wear the arm servo (hand) code will go
                if (gamepad2.dpad_down) verticalMode.bucketDown();
                if (gamepad2.dpad_right) verticalMode.ready();
                if (gamepad2.dpad_left) verticalMode.dump();
            }

            // Adjusts percentage of wheel power
            if (percent < 100 && gamepad1.dpad_up && !dpadUp) percent += 5;
            if (percent > 0 && gamepad1.dpad_down && !dpadDown) percent -= 5;
            if (gamepad1.a) percent = 20;
            if (gamepad1.y) percent = 100;
            if (gamepad1.b) percent = 65;

            // Sets the past gamepad positioning after each pass of while loop to account for newly pressed buttons
            dpadUp = gamepad1.dpad_up;
            dpadDown = gamepad1.dpad_down;

            // Calls drive function and changes inputs whether the robot is in field or robot centric mode
            double x;
            double y;
            if (fieldCentric) {
                y = newX;
                x = newY;
            }
            else {
                y = strafe;
                x = drive;
            }
            drive(x, y, rotate, percent);

            // adding telemetry data
            telemetry.addLine("Robot Data");
            // Displays the current drive mode
            telemetry.addData("Field Centric Mode", fieldCentric);
            telemetry.addData("Robot Position (X axis)", angles.firstAngle);
            telemetry.addData("Vertical Slide Mode", vertical);
            telemetry.addData("Manual Arm control: ", manual);

            telemetry.addLine("\nVertical Arm Data");
            telemetry.addData("Vertical Arm Ext Position: ", arm.getCurrentPosition());
            telemetry.addData("Vertical Bucket Position: ", hand.getController().getServoPosition(0));

            telemetry.addLine("\nHorizontal Arm Data");
            telemetry.addData("Right Servo Position", inputServo.getController().getServoPosition(inputServo.getPortNumber()));
            telemetry.addData("Horizontal Arm Pivot Position", intakeMotor.getCurrentPosition());
            telemetry.addData("Horizontal Slide Position", hSlide.getCurrentPosition());

            telemetry.addLine("\nDrive Data");
//            telemetry.addData("Drive", drive);
//            telemetry.addData("Strafe", strafe);
//            telemetry.addData("Rotate", rotate);
            telemetry.addData("Percent Power", percent);
            telemetry.addData("Left Front Power", leftFront.getPower());
            telemetry.addData("Right Front Power", rightFront.getPower());
            telemetry.addData("Left Back Power", leftBack.getPower());
            telemetry.addData("Right Back Power", rightBack.getPower());

            telemetry.update();
        } // While op mode is active
    } // Run Op Mode

    public void stopAll() {
        percent = 0;
        verticalMode.stopArm();
        horizontalMode.stopAll();
    }

    // Drive function
    public void drive(double drive, double strafe, double rotate, double percent) {
        // The percent variable is used to set the motor power to that percent
        // Algorithm adapted from ChatGPT
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = -drive + strafe + rotate;
        double backLeftPower = -drive + strafe - rotate;
        double backRightPower = drive + strafe - rotate;

        // Normalize the values so no value exceeds 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

//        if (max > 1.0 || max < 1.0) {
//            frontLeftPower /= max;
//            frontRightPower /= max;
//            backLeftPower /= max;
//            backRightPower /= max;
//        }

        if (!gamepad1.left_bumper) {
            //sets the power to an inputted percent to control sensitivity
            frontLeftPower *= percent/100;
            frontRightPower *= percent/100;
            backLeftPower *= percent/100;
            backRightPower *= percent/100;
        }

        // Output the safe vales to the motor drives.
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
    }
}
