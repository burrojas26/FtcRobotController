package org.firstinspires.ftc.teamcode;


//From ftclib
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//From normal ftc library
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp

public class updatedDriveTest extends LinearOpMode {

    //Variables
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    //Servo launcher;

    @Override

    public void runOpMode() {

        Motor leftFront = new Motor(hardwareMap, "leftFront");
        Motor rightFront = new Motor(hardwareMap, "rightFront");
        Motor leftBack = new Motor(hardwareMap, "leftBack");
        Motor rightBack = new Motor(hardwareMap, "rightBack");


        //Instance of class GamepadEx from ftclib
        //Allows for easier control of button actions
        GamepadEx gamepadOne = new GamepadEx(gamepad1);

        waitForStart();

        double percent = 0;
        while (opModeIsActive()) {
            // Variables
            double strafe = -gamepad1.left_stick_x;
            double drive = gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;

            if (gamepad1.y) {
                drive = -1;
                strafe = 0;
                rotate = 0;
            }

            if (gamepad1.a) {
                drive = 1;
                strafe = 0;
                rotate = 0;
            }

            if (gamepad1.b) {
                drive = 0;
                strafe = 1;
                rotate = 0;
            }

            if (gamepad1.x) {
                drive = 0;
                strafe = -1;
                rotate = 0;
            }

            // Allows gamepad2 to take over driving
            if (gamepad2.left_bumper) {
                strafe = -gamepad2.left_stick_x;
                drive = gamepad2.left_stick_y;
                rotate = gamepad2.right_stick_x;
            }

            //Allows the user to control the percentage of speed based on the inputs from the dpad
            if (gamepadOne.wasJustPressed(GamepadKeys.Button.DPAD_UP) && percent < 100) {
                percent+=1;
            }

            if (gamepadOne.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) && percent > 0) {
                percent-=1;
            }

            MecanumDrive driver = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);
            RevIMU imu = new RevIMU(hardwareMap);
            imu.init();


            driver.driveFieldCentric(strafe, drive, rotate, imu.getRotation2d().getDegrees());

            // adding telemetry data
            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);
            telemetry.addData("Percent", percent);
            telemetry.update();


        } // While op mode is active
    } // Run Op Mode

}
