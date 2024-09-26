package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;



@TeleOp

public class updatedDriveTest extends LinearOpMode {

    //Variables
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    //Servo launcher;

    @Override

    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");


        //Need to reverse motors on one side
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // Variables
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double percent = 0;

            // Allows gamepad2 to take over driving
            if (gamepad2.left_bumper) {
                x = gamepad2.left_stick_x;
                y = -gamepad2.left_stick_y;
                turn = gamepad2.right_stick_x;
            }

            //calculating theta and power for driving function
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            if (gamepad1.dpad_up) {
                percent+=10;
            }

            if (gamepad1.dpad_down) {
                percent-=10;
            }

            drive(theta, power, turn, percent);

            // adding telemetry data
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("turn", turn);
            telemetry.addData("theta", theta);
            telemetry.addData("power", power);
            telemetry.addData("percent speed", percent);
            telemetry.addData("Left Front Power", leftFront.getPower());
            telemetry.addData("Right Front Power", rightFront.getPower());
            telemetry.addData("Left Back Power", leftBack.getPower());
            telemetry.addData("Right Back Power", rightBack.getPower());


        } // While op mode is active
    } // Run Op Mode

    //Functions
    //https://youtu.be/gnSW2QpkGXQ?si=S0n82yAB5Zl1MYK9 (shows a more complex method for programming mechanum wheels)
    public void drive(double theta, double power, double turn, double percent) {
        //Algorithm from video above
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = power * cos/max + turn;
        double rightFrontPower = power * sin/max - turn;
        double leftBackPower = power * cos/max + turn;
        double rightBackPower = power * sin/max - turn;

        if ((power + Math.abs(turn)) > 1) {
            leftFrontPower /= power + turn;
            rightFrontPower /= power + turn;
            leftBackPower /= power + turn;
            rightBackPower /= power + turn;
        }

        leftFrontPower -= leftFrontPower*(percent/100);
        rightFrontPower -= rightFrontPower*(percent/100);
        leftBackPower -= leftBackPower*(percent/100);
        rightBackPower -= rightBackPower*(percent/100);
    }
}
