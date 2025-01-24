package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;

public class lowGripper {
    public static Servo low_gripper;
    boolean padLock = false;

    static private boolean isOpen = false;
    public lowGripper(HardwareMap hardwareMap) {
        low_gripper = hardwareMap.servo.get(ConstantNamesHardwaremap.LOWGRIPPER);
        low_gripper.setPosition(Constants.LOGRIPPER_CLOSE_POS);
        isOpen = true;
    }

    public void handleServo(Gamepad gamepad){
        if(gamepad.cross && !padLock && isOpen){
            CloseLowGripper();
            padLock = true;
        }
        else if(gamepad.cross && !padLock && !isOpen){
            OpenLowGripper();
            padLock = true;
        }
        else if(!gamepad.cross && padLock) padLock = false;
    }

    public static void OpenLowGripper(){
        low_gripper.setPosition(Constants.LOGRIPPER_OPEN_POS);
        isOpen = true;
    }

    public static void CloseLowGripper(){
        low_gripper.setPosition(Constants.LOGRIPPER_CLOSE_POS);
        isOpen = false;
    }
}