package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.TeleOp.state_25.Arm;
import org.firstinspires.ftc.teamcode.TeleOp.state_25.Auto;
import org.firstinspires.ftc.teamcode.TeleOp.state_25.Intake;

@Autonomous
public class AutoFinal extends LinearOpMode {

    //Variables
    DcMotorEx leftFront, leftBack, rightFront, rightBack; // Drivetrain motors
    DcMotorEx leftArm, rightArm; // Arm motors
    Servo leftServo, rightServo; // Arm rotation servos
    Servo pinch, rotator; // Intake servos

    // Localization and Arm control
    GoBildaPinpointDriver pinpoint;
    Arm arm;
    Intake intake;
    Auto auto;

    // Control Variables
    double percent; // Power percentage for driving
    boolean stopped = false;

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
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure motor directions (ensuring forward movement aligns with joystick input)
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        // Configure Arm motor directions
        leftArm.setDirection(DcMotorSimple.Direction.FORWARD);
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure other servo
        rotator.setDirection(Servo.Direction.REVERSE);

        // Ensuring one Arm servo moves in reverse
        leftServo.setDirection(Servo.Direction.REVERSE);

        // Configure the pinpoint localization system
        pinpoint.setOffsets(107.3, 0);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        // Reset encoders for the arm motors
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        Arm arm = new Arm(leftArm, rightArm, leftServo, rightServo, gamepad2);

        // Set rotation of arm
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);

        // Set intake starting positions
        pinch.setPosition(0);
        rotator.setPosition(.2);
        waitForStart();
        percent = 65;
        if (opModeIsActive()) {
            Intake intake = new Intake(pinch, rotator);
            Auto auto = new Auto(arm, intake);
            drive(1, 0, 0, percent);
            pause(300);
            drive(0, -1, 0, percent);
            pause(1000);
            drive(1, 0, 0, percent);
            pause(450);
            drive(0, 0, 0, percent);
            auto.scoreSample();
            pause(2000);
            intake.open();
            pause(100);
            drive(-1, 0, 0, percent);
            pause(800);
            drive(0,1, 0, percent);
            pause(1800);
            drive(0, 0, 0, percent);
            arm.collapseArm();
            arm.armUp();
//            double v = leftFront.getVelocity();
//            ElapsedTime runTime = new ElapsedTime();
//            double distance = 0.5*v*runTime.seconds();
//            telemetry.addData("Distance (Ticks)", distance);
//            telemetry.update();
            pause(10000);
        }
    }

    // Sleep Function
    public void pause(long time) {
        try {
            // Sleep for 2 seconds (2000 milliseconds)
            Thread.sleep(time);
        } catch (InterruptedException e) {
            System.out.println("Thread was interrupted!");
        }
    }

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
