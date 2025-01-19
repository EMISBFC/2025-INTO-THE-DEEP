package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GripperSpinner {
    private Servo LRot;
    private Servo RRot;
    public GripperSpinner(HardwareMap hardwareMap){
        LRot = hardwareMap.servo.get(ConstantNamesHardwaremap.GRIPPERSPINNERLEFT);
        RRot = hardwareMap.servo.get(ConstantNamesHardwaremap.GRIPPERSPINNERRIGHT);
        LRot.setDirection(Servo.Direction.REVERSE);
        LRot.setPosition(Constants.InRotPosDown);
        RRot.setDirection(Servo.Direction.FORWARD);
        RRot.setPosition(Constants.InRotPosDown);
    }

    public void handleSpinner(Gamepad gamepad) {

        if(gamepad.dpad_up) {
            LRot.setDirection(Servo.Direction.REVERSE);
            LRot.setPosition(Constants.InRotPosUp);
            RRot.setDirection(Servo.Direction.FORWARD);
            RRot.setPosition(Constants.InRotPosUp);
        }
        if(gamepad.dpad_right) {
            LRot.setDirection(Servo.Direction.REVERSE);
            LRot.setPosition(Constants.InRotPosMid);
            RRot.setDirection(Servo.Direction.FORWARD);
            RRot.setPosition(Constants.InRotPosMid);
        }
        if(gamepad.dpad_down) {
            LRot.setDirection(Servo.Direction.REVERSE);
            LRot.setPosition(Constants.InRotPosDown);
            RRot.setDirection(Servo.Direction.FORWARD);
            RRot.setPosition(Constants.InRotPosDown);
        }
        }
    }
