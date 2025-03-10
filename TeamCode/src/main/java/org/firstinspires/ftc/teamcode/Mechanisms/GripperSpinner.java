package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.Constants;

public class GripperSpinner {
    public static Servo LRot, RRot;

    public GripperSpinner(HardwareMap hardwareMap) {
        LRot = hardwareMap.get(Servo.class, "loRotL");
        RRot = hardwareMap.get(Servo.class, "loRotR");

        LRot.setDirection(Servo.Direction.REVERSE);
        RRot.setDirection(Servo.Direction.FORWARD);
    }

    public void handleInput(Gamepad gamepad){
        if (gamepad.dpad_down) {
            moveToDown();
        } else if (gamepad.dpad_left) {
            moveToMid();
        } else if (gamepad.dpad_up) {
            moveToUp();
        }
    }

    public void handleInputLeft(Gamepad gamepad){

    }

    public void moveToDown() {
        LRot.setPosition(Constants.GRIPPER_SPINNER_DOWN);
        RRot.setPosition(Constants.GRIPPER_SPINNER_DOWN);
    }

    public void moveToMid() {
        LRot.setPosition(Constants.GRIPPER_SPINNER_MID);
        RRot.setPosition(Constants.GRIPPER_SPINNER_MID);
    }

    public void moveToUp() {
        LRot.setPosition(Constants.GRIPPER_SPINNER_UP);
        RRot.setPosition(Constants.GRIPPER_SPINNER_UP);
    }
}
