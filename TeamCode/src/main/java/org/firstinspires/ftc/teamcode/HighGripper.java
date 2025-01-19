package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HighGripper {
    static public Servo high_gripper;
    static private boolean padLock = false;
    static private boolean isOpen = false;

    public HighGripper(HardwareMap hardwareMap) {
        high_gripper = hardwareMap.servo.get(ConstantNamesHardwaremap.HIGHGRIPPER);
        high_gripper.setPosition(Constants.HIGRIPPER_CLOSE_POS);
        isOpen = false;
    }

    public void handleServo(Gamepad gamepad) {
        if (gamepad.triangle && !padLock && isOpen) {
            high_gripper.setPosition(Constants.HIGRIPPER_CLOSE_POS);
            padLock = true;
            isOpen = false;
        } else if (gamepad.triangle && !padLock && !isOpen) {
            high_gripper.setPosition(Constants.HIGRIPPER_OPEN_POS);
            padLock = true;
            isOpen = true;
        } else if (!gamepad.triangle && padLock) padLock = false;
    }
}
