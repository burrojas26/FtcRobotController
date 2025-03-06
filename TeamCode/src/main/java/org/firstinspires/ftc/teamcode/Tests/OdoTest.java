package org.firstinspires.ftc.teamcode.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;

@TeleOp
@Disabled
public class OdoTest extends LinearOpMode {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;



    double percent;
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        // Initializing hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        odo.setOffsets(107.3, 0); //CHANGE THESE VALUES
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
        percent = 50;
        waitForStart();
        boolean dpadUp = false;
        boolean dpadDown = false;
        boolean dpadleft = false;
        boolean dpadright = false;
        int xDistance = 10;
        int yDistance = 10;
        while(opModeIsActive()) {
            odo.update();
            if (!dpadUp && gamepad1.dpad_up)  yDistance+=5;
            dpadUp = gamepad1.dpad_up;
            if (!dpadDown && gamepad1.dpad_down)  yDistance-=5;
            dpadDown = gamepad1.dpad_down;
            if (!dpadleft && gamepad1.dpad_left)  xDistance-=5;
            dpadleft = gamepad1.dpad_left;
            if (!dpadright && gamepad1.dpad_right)  xDistance+=5;
            dpadright = gamepad1.dpad_right;
            if (gamepad1.a) {
                drive(1, 0, 0, percent);
                while (odo.getPosition().getX(DistanceUnit.MM) < xDistance) updateDrivingTelemetry(xDistance, yDistance);
                drive(0, 1, 0, percent);
                while (odo.getPosition().getY(DistanceUnit.MM) <yDistance) updateDrivingTelemetry(xDistance, yDistance);

            }
            // Getting inputs for driving
            double strafe = gamepad1.left_stick_x;
            double drive = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;
            drive(drive, strafe, rotate, percent);
            //updateDrivingTelemetry(xDistance, yDistance);
            telemetry.addData("Current X", odo.getPosition().getX(DistanceUnit.MM));
            telemetry.addData("Current Y", odo.getPosition().getY(DistanceUnit.MM));
            telemetry.addData("Current Y", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.update();
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
    public void updateDrivingTelemetry(double xDistance, double yDistance) {
        telemetry.addData("Left Front", leftFront.getPower());
        telemetry.addData("Right Front", rightFront.getPower());
        telemetry.addData("Left Back", leftBack.getPower());
        telemetry.addData("Right Back", rightBack.getPower());
        telemetry.addData("Target X", xDistance);
        telemetry.addData("Target Y", yDistance);
        telemetry.addData("Current X", odo.getPosition().getX(DistanceUnit.MM));
        telemetry.addData("Current Y", odo.getPosition().getY(DistanceUnit.MM));
        telemetry.addData("Current Z", odo.getHeading());
        telemetry.update();
    }

}