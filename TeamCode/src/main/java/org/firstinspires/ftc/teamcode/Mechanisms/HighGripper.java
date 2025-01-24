package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;

public class HighGripper {
    static public ServoImplEx high_gripper;
    static private boolean padLock = false;
    static private boolean isOpen = false;

    public HighGripper(HardwareMap hardwareMap) {
        high_gripper = hardwareMap.get(ServoImplEx.class, ConstantNamesHardwaremap.HIGHGRIPPER);
        high_gripper.setPwmRange(new PwmControl.PwmRange(555, 2450));
        high_gripper.setPosition(Constants.HIGRIPPER_CLOSE_POS);
        isOpen = false;
    }

    public void handleServo(Gamepad gamepad) {
        if (gamepad.cross && !padLock && isOpen) {
            CloseGripper();
            padLock = true;
        } else if (gamepad.cross && !padLock && !isOpen) {
            OpenGripper();
            padLock = true;
        } else if (!gamepad.cross && padLock) padLock = false;
    }

    public static void CloseGripper(){
        high_gripper.setPosition(Constants.HIGRIPPER_CLOSE_POS);
        isOpen = false;
    }
    public static void OpenGripper(){
        high_gripper.setPosition(Constants.HIGRIPPER_OPEN_POS);
        isOpen = true;
    }
}
