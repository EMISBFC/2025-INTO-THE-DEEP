package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class GripperLearn {

    private Servo fkthis;

    private Boolean openFK = false;

    public GripperLearn(HardwareMap hardwareMap) {
        fkthis = hardwareMap.servo.get("gripper");
    }


    public void handleServo(Gamepad gamepad){
        if(gamepad.cross && !openFK){
            openFK = true;

            fkthis.setPosition(Constants.INTAKE_OPEN_POS);
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 300) {
                //do nothing
            }
        } else if (gamepad.cross && openFK) {
            openFK = false;

            fkthis.setPosition(Constants.INTAKE_CLOSE_POS);
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 300) {
                //do nothing
            }
        }
    }
}



