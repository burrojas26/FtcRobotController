/**
 * @author Jasper Burroughs
 * This is the final program for the teleop period in the 2024-25 competition (Into the deep)
 */


package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp

public class TeleOpFinal extends LinearOpMode {



    //Variables
    IMU imu;
    Orientation angles;
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    DcMotorEx arm;
    Servo hand;
    //CRServo intake;
    //DcMotorEx intakeRotate;

    @Override

    public void runOpMode() {

        //calculations for PIDF values from First Global Motor PIDF Tuning guide - values used for velocity control



        // Initializing hardware
        // Get the IMU instance
        imu = hardwareMap.get(IMU.class, "imu");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        arm = hardwareMap.get(DcMotorEx.class, "slide");
        hand = hardwareMap.get(Servo.class, "hand");
        //intake = hardwareMap.get(CRServo.class, "intake");
        //intakeRotate = hardwareMap.get(DcMotorEx.class, "intakeRotate");

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


        waitForStart();

        /* Declaring variables
        Percent is the percentage of power for driving
        PastGP is a gamepad that is reset at every run of the while loop
        */
        double percent = 65;
        boolean dpadUp = false;
        boolean dpadDown = false;
        boolean fieldCentric = false;
        hand.setDirection(Servo.Direction.FORWARD);

        while (opModeIsActive()) {

            // Getting inputs for driving
            double strafe = gamepad1.left_stick_x;
            double drive = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;

            // Displays the current drive mode
            telemetry.addData("Field Centric Mode", fieldCentric);

            // Get the robot's current heading in radians
            // Retrieve the robot's orientation
            Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double robotHeading = Math.toRadians(angles.firstAngle);
            telemetry.addData("Robot Position", angles.firstAngle);

            // Field-centric transformation using Rotation Matrix
            double newX = strafe * Math.cos(robotHeading) - drive * Math.sin(robotHeading);
            double newY = strafe * Math.sin(robotHeading) + drive * Math.cos(robotHeading);


            // Allows gamepad2 to take over driving
            if (gamepad2.left_bumper) {
                strafe = gamepad2.left_stick_x;
                drive = -gamepad2.left_stick_y;
                rotate = -gamepad2.right_stick_x;
            }

            // Toggles between field centric and robot centric
            if (gamepad1.ps) {
                fieldCentric = !fieldCentric;
            }

            //Active intake servo code
//            if (gamepad1.b) {
//                intake.setPower(1);
//            }
//            if (gamepad1.x) {
//                intake.setPower(-1);
//            }
//
//            // Intake Motor code
//            intakeRotate.setPower(0.5*gamepad2.left_stick_x);
//            telemetry.addData("Intake Position", intakeRotate.getCurrentPosition());
//
//            //Adding telemetry data
//            telemetry.addData("intake position", intake.getController().getServoPosition(1));

            // Arm extension control
            if (gamepad2.x) {
                arm.setVelocity(0);
            }
            if (gamepad2.right_bumper) {
                arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad2.y) {
                arm.setTargetPosition(-2750);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setVelocity(4000);
            }
            if(gamepad2.a) {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setVelocity(4000);
            }
            if (gamepad2.ps) {
                arm.setTargetPosition(-1900);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setVelocity(4000);
            }
            // Telemetry data for arm
            telemetry.addData("Arm Velocity: ", arm.getVelocity());
            telemetry.addData("Arm Ext Position: ", arm.getCurrentPosition());

            // This is wear the arm servo (hand) code will go
            if (gamepad2.dpad_down) {
                hand.setPosition(1);
            }
            if(gamepad2.dpad_right) {
                hand.setPosition(0.85);
            }
            if(gamepad2.dpad_left) {
                hand.setPosition(0.1);
            }
            // Telemetry data for and
            telemetry.addData("Position: ", hand.getController().getServoPosition(0));

            // Adjusts percentage of wheel power
            if (percent < 100 && gamepad1.dpad_up && !dpadUp) {
                percent += 5;
            }
            if (percent > 0 && gamepad1.dpad_down && !dpadDown) {
                percent -= 5;
            }

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
            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);
            telemetry.addData("Percent Power", percent);
            telemetry.addData("Left Front Power", leftFront.getPower());
            telemetry.addData("Right Front Power", rightFront.getPower());
            telemetry.addData("Left Back Power", leftBack.getPower());
            telemetry.addData("Right Back Power", rightBack.getPower());

            telemetry.update();
        } // While op mode is active
    } // Run Op Mode

    // Drive function
    // https://youtu.be/gnSW2QpkGXQ?si=S0n82yAB5Zl1MYK9 (shows a more complex method for programming mechanum wheels)
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

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

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
