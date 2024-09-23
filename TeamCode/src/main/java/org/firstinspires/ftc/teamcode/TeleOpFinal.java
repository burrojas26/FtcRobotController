package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;



@TeleOp

public class TeleOpFinal extends LinearOpMode {

    //Variables
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    CRServo conveyer;
    DcMotorEx sweep;
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
        conveyer = hardwareMap.get(CRServo.class, "conveyer");
        sweep = hardwareMap.get(DcMotorEx.class, "sweep");
        
        //launcher = hardwareMap.get(Servo.class, "launcher");
        
        // PIDF coefficients are for sustaining a specific velocity
        // leftFront.setVelocityPIDFCoefficients(1.057, 0.1057, 0, 10.57);
        // leftBack.setVelocityPIDFCoefficients(1.057, 0.1057, 0, 10.57);
        // rightFront.setVelocityPIDFCoefficients(1.057, 0.1057, 0, 10.57);
        // rightBack.setVelocityPIDFCoefficients(1.057, 0.1057, 0, 10.57);
        // conveyer.setVelocityPIDFCoefficients(1.22, 0.122, 0, 12.2);
        sweep.setVelocityPIDFCoefficients(1.22, 0.122, 0, 12.2);
        
        //Max V for new motors = 3100.0
        //newMotor.setVelocityPIDFCoefficients(1.057, 0.1057, 0, 10.57);
        
        //Need to reverse motors on one side
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // Variables
            boolean reset = false;
            double strafe = -gamepad1.left_stick_x;
            double drive = gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;
            
            // Allows gamepad2 to take over driving
            if (gamepad2.left_bumper) {
                strafe = -gamepad2.left_stick_x;
                drive = gamepad2.left_stick_y;
                rotate = gamepad2.right_stick_x;
            }
            
            // Controls conveyer speeds
            if (gamepad1.right_trigger < 0.80 && gamepad1.right_trigger != 0.0) {
                conveyer.setPower(gamepad1.right_trigger);
            }
            else if (gamepad1.right_trigger == 1.0) {
                conveyer.setPower(0.80);
            }
            else if (gamepad1.right_trigger < 0.03) {
                conveyer.setPower(0);
            }

            drive(drive, strafe, rotate);
            
            // adding telemetry data
            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);
            telemetry.addData("Left Front Power", leftFront.getPower());
            telemetry.addData("Right Front Power", rightFront.getPower());
            telemetry.addData("Left Back Power", leftBack.getPower());
            telemetry.addData("Right Back Power", rightBack.getPower());
            
            
            //sweep.setVelocity(conveyerSpeed);
            
            telemetry.addData("Conveyer Power", conveyer.getPower());
            
            if (gamepad1.y || gamepad2.y) {
                reset = true;
            }
            telemetry.addData("Reset", reset);
            
         /*   if (gamepad1.ps || gamepad2.ps) {
                if (launcher.getPosition() == 0 && !reset) {
                    launcher.setPosition(1);
                }
                if (launcher.getPosition() == 1 && reset) {
                    launcher.setPosition(0);
                }
            }
            
            telemetry.addData("Launcher Position", launcher.getPosition());
            
            telemetry.update();
            */
        } // While op mode is active
    } // Run Op Mode
    
    //Functions
    public void drive(double drive, double strafe, double rotate) {
        // The percent variable is used to decrease the power by that percent
        double percent = 20.0;
        // Algorithm created by ChatGPT
        double frontLeftPower = drive + strafe - rotate;
        double frontRightPower = -drive + strafe - rotate;
        double backLeftPower = -drive + strafe + rotate;
        double backRightPower = drive + strafe - rotate;

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
            frontLeftPower -= frontLeftPower*(percent/100);
            frontRightPower -= frontRightPower*(percent/100);
            backLeftPower -= backLeftPower*(percent/100);
            backRightPower -= backRightPower*(percent/100);
        }
        
        // Output the safe vales to the motor drives.
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
        telemetry.update();
    }
}
