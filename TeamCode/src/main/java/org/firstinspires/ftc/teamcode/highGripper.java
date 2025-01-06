package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class highGripper {
    public Servo high_gripper;
    boolean padLock = false;
    boolean isOpen = false;
    public highGripper(HardwareMap hardwareMap) {
        high_gripper = hardwareMap.servo.get("hiServo");
        high_gripper.setPosition(Constants.HIGRIPPER_CLOSE_POS);
        isOpen = false;
    }

    
    public void handleServo(Gamepad gamepad){

        if(gamepad.triangle && !padLock && isOpen){ // end me , ty u/4106Thumbs

            high_gripper.setPosition(Constants.HIGRIPPER_CLOSE_POS);
            padLock = true;
            isOpen = false;
        }
        else if(gamepad.triangle && !padLock && !isOpen){
            high_gripper.setPosition(Constants.HIGRIPPER_OPEN_POS);
            padLock = true;
            isOpen = true;
        }
        else if(!gamepad.triangle && padLock) padLock = false;
    }
}