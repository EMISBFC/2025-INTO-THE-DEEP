package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;

public class lowGripperTest {
    public static Servo LGrip;
    public static Servo RGrip;

    static private boolean padLock = false;
    static private boolean isOpen = false;

    public lowGripperTest(HardwareMap hardwareMap) {
        LGrip = hardwareMap.servo.get(ConstantNamesHardwaremap.GRIPPERLOWLEFT);
        RGrip = hardwareMap.servo.get(ConstantNamesHardwaremap.GRIPPERLOWRIGHT);
        LGrip.setDirection(Servo.Direction.REVERSE);
        //LRot.setPosition(Constants.InRotPosDown);
        RGrip.setDirection(Servo.Direction.FORWARD);
        //RRot.setPosition(Constants.InRotPosDown);
    }

    public void lowGripperTest(Gamepad gamepad) {
        if (gamepad.cross && !padLock && isOpen) {
            CloseLowGripper();
            padLock = true;
        } else if (gamepad.cross && !padLock && !isOpen) {
            OpenLowGripper();
            padLock = true;
        } else if (!gamepad.cross && padLock) padLock = false;
    }

    public static void OpenLowGripper() {
        LGrip.setPosition(0.5);
        RGrip.setPosition(0.5);
        isOpen = true;
    }

    public static void CloseLowGripper() {
        LGrip.setPosition(0.3);
        RGrip.setPosition(0.3);
        isOpen = false;
    }
}