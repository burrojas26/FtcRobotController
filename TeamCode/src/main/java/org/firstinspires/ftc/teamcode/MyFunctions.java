package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class MyFunctions {
    // Returns true or false whether the passed in button was just pressed
    boolean justPressed(Gamepad gamepad, GamepadKeys.Button b) {
        Gamepad past = null;
        past.copy(gamepad);
        return gamepad.b && !past.b;
    }
}
