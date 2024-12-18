package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {

    private Servo gripperServo; // Declare the servo
    private final double openPosition = Constants.GRIPPER_OPEN_POSITION; // Adjust based on your hardware setup
    private final double closePosition = Constants.GRIPPER_CLOSE_POSITION ; // Adjust based on your hardware setup
    private final double pause = Constants.GRIPPER_PAUSE;
    private boolean open = false;
    public Gripper(HardwareMap hardwareMap){
        gripperServo = hardwareMap.servo.get("gripper");
    }
    public void handleServo(Gamepad gamepad) {

        if (gamepad.cross && open == false) {
            open = true;
            gripperServo.setPosition(openPosition);
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < pause) {
                //do nothing. YES I KNOW QUALITY CODE
            }
        } else if (gamepad.cross && open == true) {
            open = false;
            gripperServo.setPosition(closePosition);
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < pause) {
                //do nothing. YES I KNOW QUALITY CODE
            }
        }
    }

    public void setOpenPosition() {
        open = true;
        gripperServo.setPosition(openPosition);
    }

    public void setClosePositionGripper() {
        open = false;
        gripperServo.setPosition(closePosition);
    }
}
