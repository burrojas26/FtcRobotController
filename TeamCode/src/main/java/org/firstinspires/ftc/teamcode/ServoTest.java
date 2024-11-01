package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class ServoTest extends LinearOpMode {

    //Variables
    //Linear slide motor may not be a DcMotorEx
    Servo hand;

    @Override

    public void runOpMode() {
        hand = hardwareMap.get(Servo.class, "hand");

        waitForStart();
        hand.setPosition(0.7);
        while (opModeIsActive()) {

            if (gamepad1.b) {
                hand.setPosition(0.85);
            }
            if(gamepad1.y) {
                hand.setDirection(Servo.Direction.FORWARD);
                hand.setPosition(0.7);
            }
            if(gamepad1.a) {
                hand.setDirection(Servo.Direction.FORWARD);
                hand.setPosition(0);
            }

            telemetry.addData("Position: ", hand.getController().getServoPosition(0));
            telemetry.update();
        }

    }
}

