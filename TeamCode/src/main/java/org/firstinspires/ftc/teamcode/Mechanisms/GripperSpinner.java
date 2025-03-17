package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Autos.LeftAuto;
import org.firstinspires.ftc.teamcode.Constants.Constants;

public class GripperSpinner {
    public static ServoImplEx LRot, RRot;
    static private boolean padLock = false;
    static private boolean isOpen = false;

    public GripperSpinner(HardwareMap hardwareMap) {
        LRot = hardwareMap.get(ServoImplEx.class, "loRotL");
        RRot = hardwareMap.get(ServoImplEx.class, "loRotR");
        LRot.setPwmRange(new PwmControl.PwmRange(500,2500));
        RRot.setPwmRange(new PwmControl.PwmRange(500,2500));

        LRot.setDirection(Servo.Direction.REVERSE);
        RRot.setDirection(Servo.Direction.FORWARD);
    }

    public void handleInput(Gamepad gamepad) {

        if(gamepad.triangle && !padLock && isOpen) {
            moveToMid();
            isOpen = false;
            padLock = true;
        }
        else if(gamepad.triangle && !padLock&& !isOpen) {
            moveToDown();
            isOpen = true;
            padLock = true;
        }
        else if(!gamepad.triangle && padLock) padLock = false;
    }

    /*public void handleInput(Gamepad gamepad){
        if (gamepad.dpad_down) {
            moveToDown();
        } else if (gamepad.dpad_left) {
            moveToMid();
        } else if (gamepad.dpad_up) {
            moveToUp();
        }
    }
*/
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

    // actions
    public class Down implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            LRot.setDirection(Servo.Direction.REVERSE);
            LRot.setPosition(Constants.InRotPosDown);
            RRot.setDirection(Servo.Direction.FORWARD);
            RRot.setPosition(Constants.InRotPosDown);
            return false;
        }
    }

    public class Up implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            LRot.setDirection(Servo.Direction.REVERSE);
            LRot.setPosition(Constants.InRotPosUp);
            RRot.setDirection(Servo.Direction.FORWARD);
            RRot.setPosition(Constants.InRotPosUp);
            return false;
        }
    }

    public class Mid implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            LRot.setDirection(Servo.Direction.REVERSE);
            LRot.setPosition(Constants.InRotPosMid);
            RRot.setDirection(Servo.Direction.FORWARD);
            RRot.setPosition(Constants.InRotPosMid);
            return false;
        }

    }

    public Action down() {
        return new Down();
    }

    public Action mid() {
        return new Mid();
    }

    public Action up() {
        return new Up();
    }
}
