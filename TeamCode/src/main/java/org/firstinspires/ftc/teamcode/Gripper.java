package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {

    private Servo gripper;
    boolean crossLock = false;
    boolean isOpen = false;
    public Gripper(HardwareMap hardwareMap) {
        gripper = hardwareMap.servo.get("inServo");
    }

    public void handleServo(Gamepad gamepad){

        if(gamepad.cross && !crossLock && isOpen){ // end me , ty u/4106Thumbs
//            gripper.setDirection(Servo.Direction.REVERSE);
            gripper.setPosition(Constants.INTAKE_CLOSE_POS);
            crossLock = true;
            isOpen = false;
        }
        else if(gamepad.cross && !crossLock && !isOpen){
//            gripper.setDirection(Servo.Direction.REVERSE);
            gripper.setPosition(Constants.INTAKE_OPEN_POS);
            crossLock = true;
            isOpen = true;
        }
        else if(!gamepad.cross && crossLock) crossLock = false;

 /*       if(gamepad.cross && !openFK){
            openFK = true;

            fkthis.setPosition(Constants.GRIPPER_OPEN_POSITION);
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 300) {
                //do nothing
            }
        } else if (gamepad.cross && openFK) {
            openFK = false;

            fkthis.setPosition(Constants.GRIPPER_CLOSE_POSITION);
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 300) {
                //do nothing
            } */
    }
}

