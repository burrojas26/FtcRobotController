package org.firstinspires.ftc.teamcode.TeleOp.state_25;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    Servo leftServo;
    Servo rightServo;
    double servoMax = 0; // TODO
    double servoMin = 0; // TODO

    public Arm(DcMotorEx leftMotor, DcMotorEx rightMotor, Servo leftServo, Servo rightServo) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.leftServo = leftServo;
        this.rightServo = rightServo;
    }

    public void rotateArm(double joyStick) {
        leftServo.setPosition(leftServo.getPosition()+joyStick*0.05);
        rightServo.setPosition(rightServo.getPosition()+joyStick*0.05);
    }


}
