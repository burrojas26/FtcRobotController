package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

@Autonomous
public class AutoFinal extends LinearOpMode {

    //Variables
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    DcMotorEx arm;
    Servo hand;
    CRServo intake;
    DcMotorEx intakeRotate;

    @Override
    public void runOpMode() {
        // Initializing hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        arm = hardwareMap.get(DcMotorEx.class, "slide");
        hand = hardwareMap.get(Servo.class, "hand");
        intake = hardwareMap.get(CRServo.class, "intake");
        intakeRotate = hardwareMap.get(DcMotorEx.class, "intakeRotate");

        // All motors facing forward for the most recent build of the robot (go builda kit)
        // Some chassis builds require reversal of two of the four motors
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        waitForStart();
        double percent = 65;
        if (opModeIsActive()) {
//            long stepOne = System.currentTimeMillis() + 2500;
//            long stepTwo = stepOne + 2000;
//            while (System.currentTimeMillis() < stepOne) {
//                drive(1, 0, 0, percent);
//            }
            drive(-1, 0, 0, percent);
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
