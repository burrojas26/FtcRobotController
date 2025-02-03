package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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


public class VerticalMode {

//    DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "slide");
//    Servo hand = hardwareMap.get(Servo.class, "hand");
    DcMotorEx arm;
    Servo hand;
    Gamepad gamepad2;

    public VerticalMode(DcMotorEx arm, Servo hand, Gamepad gamepad2) {
        this.arm = arm;
        this.hand = hand;
        this.gamepad2 = gamepad2;
    }
    int vSlideMax = -2625;
    int hangPos = -1900;

    public void stopArm() {
        arm.setVelocity(0);
        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(50);
    }

    public void extendArm() {
        arm.setTargetPosition(vSlideMax);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(4000);
    }

    public void collapseArm() {
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(4000);
    }

    public void hang() {
        arm.setTargetPosition(hangPos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(4000);
    }

    public void manual() {
        float increase = gamepad2.right_stick_y * 500;
        if (arm.getCurrentPosition() + increase > vSlideMax && arm.getCurrentPosition() + increase < 0) {
            arm.setTargetPosition((int) (arm.getCurrentPosition() + increase));
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setVelocity(4000);
        }
    }

    public void resetEncode() {
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ready() {
        hand.setPosition(0.75);
    }

    public void bucketDown() {
        hand.setPosition(1);
    }

    public void dump() {
        hand.setPosition(0.25);
    }

}
