/**
 * @author Jasper Burroughs
 * This is the final program for the teleop period in the 2024-25 state competition (Into the deep)
 */


package org.firstinspires.ftc.teamcode.TeleOp.state_25;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;

@TeleOp(name="State Comp 2025", group="Linear OpMode")
public class State extends LinearOpMode {
    //Variables
    Orientation angles;
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    double percent;
    GoBildaPinpointDriver pinpoint;

    @Override

    public void runOpMode() {
        // Initializing hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"odo");


        // All motors facing forward for the most recent build of the robot (go builda kit)
        // Some chassis builds require reversal of two of the four motors
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        // Configuring the pinpoint computer
        pinpoint.setOffsets(107.3, 0);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        waitForStart();

        /* Declaring variables
        Percent is the percentage of power for driving
        PastGP is a gamepad that is reset at every run of the while loop
        */
        percent = 65;
        boolean dpadUp = false;
        boolean dpadDown = false;
        boolean fieldCentric = false;
        boolean bBtn = false;
        boolean rtStickBtn = false;
        boolean vertical = true;
        boolean manual = false;
        boolean ps = false;
        while (opModeIsActive()) {
            // Refresh the pinpoint computer
            pinpoint.update();

            // Getting inputs for driving
            double strafe = gamepad1.left_stick_x;
            double drive = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;


            // Get the robot's current heading in radians
            double robotHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

            // Field-centric transformation using Rotation Matrix
            double newX = strafe * Math.cos(robotHeading) + drive * Math.sin(robotHeading);
            double newY = -strafe * Math.sin(robotHeading) + drive * Math.cos(robotHeading);

            // Automatically stops everything
            if (gamepad1.back || gamepad2.back) stopAll();

            // Allows gamepad2 to take over driving
            if (gamepad2.left_bumper) {
                strafe = gamepad2.left_stick_x;
                drive = -gamepad2.left_stick_y;
                rotate = gamepad2.right_stick_x;
            }

            // Toggles between field centric and robot centric
            if (gamepad1.ps && !ps) {
                fieldCentric = !fieldCentric;
            }
            ps = gamepad1.ps;
//            // Switch the mode to horizontal arm
//            if (gamepad2.b && !bBtn) {
//                vertical = !vertical;
//                manual = false;
//            }
//            bBtn = gamepad2.b;

//            // Switch the arm mode to automatic
//            if (gamepad2.right_stick_button && !rtStickBtn) {
//                manual = !manual;
//            }
//            rtStickBtn = gamepad2.right_stick_button;


            // Adjusts percentage of wheel power
            if (percent < 100 && gamepad1.dpad_up && !dpadUp) percent += 5;
            if (percent > 0 && gamepad1.dpad_down && !dpadDown) percent -= 5;
            if (gamepad1.a) percent = 20;
            if (gamepad1.y) percent = 100;
            if (gamepad1.b) percent = 65;

            // Sets the past gamepad positioning after each pass of while loop to account for newly pressed buttons
            dpadUp = gamepad1.dpad_up;
            dpadDown = gamepad1.dpad_down;

            // Calls drive function and changes inputs whether the robot is in field or robot centric mode
            double x;
            double y;
            if (fieldCentric) {
                y = newX;
                x = newY;
            }
            else {
                y = strafe;
                x = drive;
            }
            drive(x, y, rotate, percent);

            // adding telemetry data
            telemetry.addLine("Robot Data");
            // Displays the current drive mode
            telemetry.addData("Field Centric Mode", fieldCentric);
            telemetry.addData("Robot Position (X axis in CM)", pinpoint.getPosition().getX(DistanceUnit.CM));
            telemetry.addData("Robot Position (Y axis in CM)", pinpoint.getPosition().getY(DistanceUnit.CM));
            telemetry.addData("Robot Position (H axis in Degrees)", pinpoint.getPosition().getHeading(AngleUnit.DEGREES));

            telemetry.addLine("\nDrive Data");
            telemetry.addData("Percent Power", percent);
            telemetry.addData("Left Front Power", leftFront.getPower());
            telemetry.addData("Right Front Power", rightFront.getPower());
            telemetry.addData("Left Back Power", leftBack.getPower());
            telemetry.addData("Right Back Power", rightBack.getPower());

            telemetry.update();
        } // While op mode is active
    } // Run Op Mode

    // Stops all movement on the robot
    public void stopAll() {
        percent = 0;
    }

    // Drive function
    public void drive(double drive, double strafe, double rotate, double percent) {
        // The percent variable is used to set the motor power to that percent
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = -drive + strafe + rotate;
        double backLeftPower = -drive + strafe - rotate;
        double backRightPower = drive + strafe - rotate;


        if (!gamepad1.left_bumper) {
            // sets the power to an inputted percent to control sensitivity
            frontLeftPower *= percent/100;
            frontRightPower *= percent/100;
            backLeftPower *= percent/100;
            backRightPower *= percent/100;
        }

        // Output the values to the motor drives.
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
    }
}
