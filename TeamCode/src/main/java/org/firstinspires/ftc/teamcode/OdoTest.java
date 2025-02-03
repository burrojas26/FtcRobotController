package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

public class OdoTest extends LinearOpMode {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    DcMotorEx arm;
    Servo hand;
    Servo inputServo;
    DcMotorEx intakeMotor;
    DcMotorEx hSlide;

    double percent;
    HorizontalMode horizontalMode = new HorizontalMode();
    VerticalMode verticalMode = new VerticalMode();
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        // Initializing hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        arm = hardwareMap.get(DcMotorEx.class, "slide");
        hand = hardwareMap.get(Servo.class, "hand");
        inputServo = hardwareMap.get(Servo.class, "inputServo");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        hSlide = hardwareMap.get(DcMotorEx.class, "hSlide");

        odo.setOffsets(-84.0, -168.0); //CHANGE THESE VALUES
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        waitForStart();
        boolean dpadUp = false;
        boolean dpadDown = false;
        boolean dpadleft = false;
        boolean dpadright = false;
        int xDistance = 10;
        int yDistance = 10;
        while(opModeIsActive()) {
            if (!dpadUp && gamepad1.dpad_up)  yDistance+=5;
            if (!dpadDown && gamepad1.dpad_down)  yDistance-=5;
            if (!dpadleft && gamepad1.dpad_left)  xDistance-=5;
            if (!dpadright && gamepad1.dpad_right)  xDistance+=5;
            if (gamepad1.a) {
                drive(1, 0, 0, 50);
                while (odo.getPosX() < xDistance) continue;
                drive(0, 1, 0, 50);
                while (odo.getPosY() <yDistance) continue;
            }
        }
    }
    // Drive function
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

//        if (max > 1.0 || max < 1.0) {
//            frontLeftPower /= max;
//            frontRightPower /= max;
//            backLeftPower /= max;
//            backRightPower /= max;
//        }

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