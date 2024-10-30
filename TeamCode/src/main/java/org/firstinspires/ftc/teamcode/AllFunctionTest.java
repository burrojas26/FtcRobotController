/**
 * @author Jasper Burroughs
 * This file tests all of the functions of the robot to ensure everything works properly
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp

public class AllFunctionTest extends LinearOpMode {

    //Variables
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    DcMotorEx arm;

    @Override

    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        arm = hardwareMap.get(DcMotorEx.class, "slide");


        //Need to reverse motors on one side
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        // Setting the position of the arm to 0 at initialization
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        double percent = 65;
        if (opModeIsActive()) {
            telemetry.addData("Testing", "Arm Extension");
            telemetry.update();

            arm.setTargetPosition(-2800);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setVelocity(4000);
            try {
                Thread.sleep(20000); // Sleep for 10,000 milliseconds (10 seconds)
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setVelocity(4000);
            try {
                Thread.sleep(20000); // Sleep for 10,000 milliseconds (10 seconds)
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            telemetry.addData("Testing", "Left Front Motor");
            telemetry.update();
            leftFront.setPower(1);
            try {
                Thread.sleep(20000); // Sleep for 10,000 milliseconds (10 seconds)
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            leftFront.setPower(0);

            telemetry.addData("Testing", "Right Front Motor");
            telemetry.update();
            rightFront.setPower(1);
            try {
                Thread.sleep(20000); // Sleep for 10,000 milliseconds (10 seconds)
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            rightFront.setPower(0);

            telemetry.addData("Testing", "Left Back Motor");
            telemetry.update();
            leftBack.setPower(1);
            try {
                Thread.sleep(20000); // Sleep for 10,000 milliseconds (10 seconds)
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            leftBack.setPower(0);

            telemetry.addData("Testing", "Right Back Motor");
            telemetry.update();
            rightBack.setPower(1);
            try {
                Thread.sleep(20000); // Sleep for 10,000 milliseconds (10 seconds)
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            rightBack.setPower(0);

            telemetry.addData("Testing", "Complete");
            telemetry.addData("Arm Velocity: ", arm.getVelocity());
            telemetry.addData("Arm Ext Position: ", arm.getCurrentPosition());
            telemetry.addData("Left Front Power", leftFront.getPower());
            telemetry.addData("Right Front Power", rightFront.getPower());
            telemetry.addData("Left Back Power", leftBack.getPower());
            telemetry.addData("Right Back Power", rightBack.getPower());

            telemetry.update();
        } // While op mode is active
    } // Run Op Mode
}
