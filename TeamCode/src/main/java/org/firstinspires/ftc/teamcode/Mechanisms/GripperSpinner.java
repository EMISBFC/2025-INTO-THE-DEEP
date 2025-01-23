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
    private boolean clicked;
    public GripperSpinner(HardwareMap hardwareMap){
        LRot = hardwareMap.servo.get(ConstantNamesHardwaremap.GRIPPERSPINNERLEFT);
        RRot = hardwareMap.servo.get(ConstantNamesHardwaremap.GRIPPERSPINNERRIGHT);
        LRot.setDirection(Servo.Direction.REVERSE);
        //LRot.setPosition(Constants.InRotPosDown);
        RRot.setDirection(Servo.Direction.FORWARD);
        //RRot.setPosition(Constants.InRotPosDown);
    }

    public void handleSpinnerRight(Gamepad gamepad) {

        if(gamepad.triangle && clicked) {
            LRot.setDirection(Servo.Direction.REVERSE);
            LRot.setPosition(Constants.InRotPosDown);
            RRot.setDirection(Servo.Direction.FORWARD);
            RRot.setPosition(Constants.InRotPosDown);
            clicked = false;
            padLock = true;
        }
        else if(gamepad.triangle && !clicked) {
            LRot.setDirection(Servo.Direction.REVERSE);
            LRot.setPosition(Constants.InRotPosMid);
            RRot.setDirection(Servo.Direction.FORWARD);
            RRot.setPosition(Constants.InRotPosMid);
            clicked = true;
            padLock = true;
        }
        else if(!gamepad.cross && padLock) padLock = false;
    }

    public void handleSpinnerLeft(Gamepad gamepad) {
        if(gamepad.triangle && clicked) {
            LRot.setDirection(Servo.Direction.REVERSE);
            LRot.setPosition(Constants.InRotPosDown);
            RRot.setDirection(Servo.Direction.FORWARD);
            RRot.setPosition(Constants.InRotPosDown);
            clicked = false;
        }
        else if(gamepad.triangle && !clicked) {
            LRot.setDirection(Servo.Direction.REVERSE);
            LRot.setPosition(Constants.InRotPosUp);
            RRot.setDirection(Servo.Direction.FORWARD);
            RRot.setPosition(Constants.InRotPosUp);
            clicked = true;
        }
    }
}
