package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp

public class MotorTest extends LinearOpMode {

    //Variables
    //Linear slide motor may not be a DcMotorEx
    DcMotorEx motor;

    @Override

    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "slide");
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {
                motor.setVelocity(0);
            }
            if(gamepad1.y) {
                motor.setTargetPosition(-2800);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setVelocity(4000);
            }
            if(gamepad1.a) {
                motor.setTargetPosition(0);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setVelocity(4000);
            }

            telemetry.addData("Velocity: ", motor.getVelocity());
            telemetry.addData("Ticks: ", motor.getCurrentPosition());
            telemetry.update();
        }

    }
}

