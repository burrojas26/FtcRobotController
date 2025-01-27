package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class HorizontalMode {

    Servo inputServo = hardwareMap.get(Servo .class, "inputServo");
    Servo hand = hardwareMap.get(Servo.class, "hand");
    DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx .class, "intakeMotor");
    DcMotorEx hSlide = hardwareMap.get(DcMotorEx.class, "hSlide");

    int hSlideMax = -12500;
    int hSlideStop = -2752;

    public void activateIntake() {
        hand.setPosition(0.92);
        hSlide.setTargetPosition(hSlideStop);
        hSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hSlide.setVelocity(6000);
        pause(3000);
        intakeMotor.setTargetPosition(-250);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setVelocity(200);
        pause(1500);
        inputServo.getController().setServoPosition(inputServo.getPortNumber(), 0);
        pause(1500);
        intakeMotor.setTargetPosition(-100);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setVelocity(600);
    }

    public void stopIntake() {
        intakeMotor.setVelocity(0);
        intakeMotor.setTargetPosition(intakeMotor.getCurrentPosition());
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setVelocity(50);
    }

    public void intakeManual() {
        float increase = gamepad2.left_stick_y * 50;
        intakeMotor.setTargetPosition((int) (intakeMotor.getCurrentPosition() + increase));
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setVelocity(400);
    }

    public void resetIntake() {
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stopExt () {
        hSlide.setVelocity(0);
        hSlide.setTargetPosition(hSlide.getCurrentPosition());
        hSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hSlide.setVelocity(50);
    }

    public void resetExt() {
        hSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void extendArm() {
        hSlide.setTargetPosition(hSlideMax);
        hSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hSlide.setVelocity(6000);
    }

    public void collapseArm() {
        hSlide.setTargetPosition(0);
        hSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hSlide.setVelocity(6000);
    }

    public void manual() {
        float increase = gamepad2.right_stick_y * 500;
        if (hSlide.getCurrentPosition() + increase > hSlideMax && hSlide.getCurrentPosition() + increase < 0) {
            hSlide.setTargetPosition((int) (hSlide.getCurrentPosition() + increase));
            hSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hSlide.setVelocity(6000);
        }
    }

    public void stopSpin() {
        inputServo.getController().setServoPosition(inputServo.getPortNumber(), inputServo.getPosition());
    }

    public void stopAll() {
        stopSpin();
        stopIntake();
        stopExt();
    }

    // Sleep Function
    private void pause(long time) {
        try {
            // Sleep for 2 seconds (2000 milliseconds)
            Thread.sleep(time);
        } catch (InterruptedException e) {
            System.out.println("Thread was interrupted!");
        }
    }

}
