/**
 * @author Jasper Burroughs
 * This is the final program for the teleop period in the 2024-25 competition (Into the deep)
 */


package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp

public class TeleOpFinal extends LinearOpMode {

    //Variables
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    DcMotorEx arm;
    //Servo launcher;

    @Override

    public void runOpMode() {
        
        //calculations for PIDF values from First Global Motor PIDF Tuning guide
        
        //Max V for old motors = 2680.0
        //motor.setVelocityPIDFCoefficients(1.22, 0.122, 0, 12.2);
        
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        arm = hardwareMap.get(DcMotorEx.class, "slide");

        
        // PIDF coefficients are for sustaining a specific velocity
        // leftFront.setVelocityPIDFCoefficients(1.057, 0.1057, 0, 10.57);
        // leftBack.setVelocityPIDFCoefficients(1.057, 0.1057, 0, 10.57);
        // rightFront.setVelocityPIDFCoefficients(1.057, 0.1057, 0, 10.57);
        // rightBack.setVelocityPIDFCoefficients(1.057, 0.1057, 0, 10.57)
        
        //Max V for new motors = 3100.0
        //newMotor.setVelocityPIDFCoefficients(1.057, 0.1057, 0, 10.57);
        
        //Need to reverse motors on one side
        leftFront.setDirection(DcMotorEx.Direction.FORWARD); //reverse
        rightFront.setDirection(DcMotorEx.Direction.FORWARD); //reverse
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        // Setting the position of the arm to 0 at initialization
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        double percent = 15;
        while (opModeIsActive()) {
            // Getting inputs
            double strafe = gamepad1.left_stick_x;
            double drive = -gamepad1.left_stick_y;
            double rotate = -gamepad1.right_stick_x;


            // Allows gamepad2 to take over driving
            if (gamepad2.left_bumper) {
                strafe = gamepad2.left_stick_x;
                drive = -gamepad2.left_stick_y;
                rotate = -gamepad2.right_stick_x;
            }

            // Arm extension control
            if (gamepad2.x) {
                arm.setVelocity(0);
            }
            if(gamepad2.y) {
                arm.setTargetPosition(-2800);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setVelocity(4000);
            }
            if(gamepad2.a) {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setVelocity(4000);
            }
            // Telemetry data for arm
            telemetry.addData("Arm Velocity: ", arm.getVelocity());
            telemetry.addData("Arm Ext Position: ", arm.getCurrentPosition());

            // Adjusts percentage of wheel power using my function in other class
            MyFunctions funcs = new MyFunctions();
            if (percent < 100 && funcs.justPressed(gamepad1, GamepadKeys.Button.DPAD_UP)) {
                percent += 5;
            }
            if (percent > 0 && funcs.justPressed(gamepad1, GamepadKeys.Button.DPAD_DOWN)) {
                percent -= 5;
            }

            // Calls drive function
            drive(drive, strafe, rotate, percent);
            
            // adding telemetry data
            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);
            telemetry.addData("Percent Power", percent);
            telemetry.addData("Left Front Power", leftFront.getPower());
            telemetry.addData("Right Front Power", rightFront.getPower());
            telemetry.addData("Left Back Power", leftBack.getPower());
            telemetry.addData("Right Back Power", rightBack.getPower());

            telemetry.update();
        } // While op mode is active
    } // Run Op Mode
    
    //Functions
    //https://youtu.be/gnSW2QpkGXQ?si=S0n82yAB5Zl1MYK9 (shows a more complex method for programming mechanum wheels)
    public void drive(double drive, double strafe, double rotate, double percent) {
        // The percent variable is used to decrease the power by that percent
        // Algorithm adapted from ChatGPT
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = -drive - strafe + rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = -drive + strafe + rotate;

        // Normalize the values so no value exceeds 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        if (!gamepad1.left_stick_button) {
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
        telemetry.update();
    }
}
