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
            Down();
            padLock = true;
        }
        else if(gamepad.triangle && !padLock&& !isOpen) {
            Mid();
            padLock = true;
        }
        else if(!gamepad.triangle && padLock) padLock = false;
    }

    public static void Down(){
        LRot.setDirection(Servo.Direction.REVERSE);
        LRot.setPosition(Constants.InRotPosDown);
        RRot.setDirection(Servo.Direction.FORWARD);
        RRot.setPosition(Constants.InRotPosDown);
        isOpen = false;
    }

    public static void Mid(){
        LRot.setDirection(Servo.Direction.REVERSE);
        LRot.setPosition(Constants.InRotPosMid);
        RRot.setDirection(Servo.Direction.FORWARD);
        RRot.setPosition(Constants.InRotPosMid);
        isOpen = true;
    }

    public static void Up(){
        LRot.setDirection(Servo.Direction.REVERSE);
        LRot.setPosition(Constants.InRotPosUp);
        RRot.setDirection(Servo.Direction.FORWARD);
        RRot.setPosition(Constants.InRotPosUp);
    }
    public void handleSpinnerLeft(Gamepad gamepad) {
        if(gamepad.triangle && !padLock && isOpen) {
           Down();
            padLock = true;
        }
        else if(gamepad.triangle && !padLock&&!isOpen) {
            Mid();
            padLock = true;
        }
        else if(!gamepad.triangle && padLock) padLock = false;
    }
}
