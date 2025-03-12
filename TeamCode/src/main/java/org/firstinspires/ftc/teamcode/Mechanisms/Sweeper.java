package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;


public class Sweeper {
    public static Servo Sweep;
    long sweepStart;
    boolean padLock = false, isOpen = false;

    public Sweeper(HardwareMap hardwareMap) {
        Sweep = hardwareMap.get(Servo.class, ConstantNamesHardwaremap.SWEEPER);
    }

    public void handleSweeper(Gamepad gamepad){
        if (gamepad.triangle) {
            Sweep();
        }
//        if (gamepad.circle && !padLock && isOpen) {
//            ToggleSweeper();
//            padLock = true;
//        }

        else if (!gamepad.cross && padLock) padLock = false;
        if (System.currentTimeMillis() - sweepStart > 200) Sweep.setPosition(Constants.sweepStart);
    }

    public void Sweep() {
        Sweep.setPosition(Constants.sweepEnd);
         sweepStart = System.currentTimeMillis();
    }

    public void ToggleSweeper() {
        if (isOpen){
            Sweep.setPosition(Constants.sweepStart);
            isOpen = false;
        }
        else if (!isOpen) {
            Sweep.setPosition(Constants.sweepEnd);
            isOpen = true;
        }
    }
}