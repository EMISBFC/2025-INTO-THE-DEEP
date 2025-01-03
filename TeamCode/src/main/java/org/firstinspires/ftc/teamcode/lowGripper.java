package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class lowGripper {
    private Servo low_gripper;
    boolean padLock = false;
    boolean isOpen = false;
    public lowGripper(HardwareMap hardwareMap) {
        low_gripper = hardwareMap.servo.get("loServo");
        low_gripper.setPosition(Constants.LOGRIPPER_OPEN_POS);
        isOpen = true;
    }

    public void handleServo(Gamepad gamepad){

        if(gamepad.cross && !padLock && isOpen){ // end me , ty u/4106Thumbs
//            gripper.setDirection(Servo.Direction.REVERSE);
            low_gripper.setPosition(Constants.LOGRIPPER_CLOSE_POS);
            padLock = true;
            isOpen = false;
        }
        else if(gamepad.cross && !padLock && !isOpen){
//            gripper.setDirection(Servo.Direction.REVERSE);
            low_gripper.setPosition(Constants.LOGRIPPER_OPEN_POS);
            padLock = true;
            isOpen = true;
        }
        else if(!gamepad.cross && padLock) padLock = false;
    }
}