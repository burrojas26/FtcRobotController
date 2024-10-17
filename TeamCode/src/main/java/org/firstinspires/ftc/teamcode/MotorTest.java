package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        //Code for controlling the motor
    }
}