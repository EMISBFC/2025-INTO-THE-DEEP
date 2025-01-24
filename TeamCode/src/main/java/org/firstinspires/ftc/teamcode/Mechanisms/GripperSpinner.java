package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;

public class GripperSpinner {
    public static Servo LRot;
    public static Servo RRot;

    static private boolean padLock = false;
    static private boolean isOpen = false;
    public GripperSpinner(HardwareMap hardwareMap){
        LRot = hardwareMap.servo.get(ConstantNamesHardwaremap.GRIPPERSPINNERLEFT);
        RRot = hardwareMap.servo.get(ConstantNamesHardwaremap.GRIPPERSPINNERRIGHT);
        LRot.setDirection(Servo.Direction.REVERSE);
        //LRot.setPosition(Constants.InRotPosDown);
        RRot.setDirection(Servo.Direction.FORWARD);
        //RRot.setPosition(Constants.InRotPosDown);
    }

    public void handleSpinnerRight(Gamepad gamepad) {

        if(gamepad.triangle && !padLock && isOpen) {
            LRot.setDirection(Servo.Direction.REVERSE);
            LRot.setPosition(Constants.InRotPosDown);
            RRot.setDirection(Servo.Direction.FORWARD);
            RRot.setPosition(Constants.InRotPosDown);
            isOpen = false;
            padLock = true;
        }
        else if(gamepad.triangle && !padLock&& !isOpen) {
            LRot.setDirection(Servo.Direction.REVERSE);
            LRot.setPosition(Constants.InRotPosMid);
            RRot.setDirection(Servo.Direction.FORWARD);
            RRot.setPosition(Constants.InRotPosMid);
            isOpen = true;
            padLock = true;
        }
        else if(!gamepad.triangle && padLock) padLock = false;
    }

    public static void fuckThis(){
        LRot.setDirection(Servo.Direction.REVERSE);
        LRot.setPosition(Constants.InRotPosDown);
        RRot.setDirection(Servo.Direction.FORWARD);
        RRot.setPosition(Constants.InRotPosDown);
        isOpen = false;
    }

    public static void fuckThat(){
        LRot.setDirection(Servo.Direction.REVERSE);
        LRot.setPosition(Constants.InRotPosMid);
        RRot.setDirection(Servo.Direction.FORWARD);
        RRot.setPosition(Constants.InRotPosMid);
        isOpen = true;
    }
    public void handleSpinnerLeft(Gamepad gamepad) {
        if(gamepad.triangle && !padLock && isOpen) {
           fuckThis();
            padLock = true;
        }
        else if(gamepad.triangle && !padLock&&!isOpen) {
            fuckThat();
            padLock = true;
        }
        else if(!gamepad.triangle && padLock) padLock = false;
    }
}
