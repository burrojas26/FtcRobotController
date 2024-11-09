package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp

public class ServoTest extends LinearOpMode {

    //Variables
    //Linear slide motor may not be a DcMotorEx
    CRServo intake;

    @Override

    public void runOpMode() {
        intake = hardwareMap.get(CRServo.class, "intake");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.b) {
                intake.setPower(1);
            }
            if (gamepad1.x) {
                intake.setPower(-1);
            }
            if (gamepad1.a) {
                intake.setPower(0);
            }


            telemetry.addData("power: ", intake.getPower());
            telemetry.addData("position", intake.getController().getServoPosition(1));
            telemetry.update();
        }

    }
}

