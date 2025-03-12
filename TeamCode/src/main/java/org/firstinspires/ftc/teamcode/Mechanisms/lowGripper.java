package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;

import java.util.List;

public class lowGripper {
    private Servo lowGripperR;
    private Servo lowGripperL;
    private boolean gripperLock = false;

    public enum GripperState {
        OPEN(Constants.LOW_GRIPPER_OPENED),
        CLOSED(Constants.LOW_GRIPPER_CLOSED);

        public final double position;
        GripperState(double position) {
            this.position = position;
        }
    }

    private GripperState currentState = GripperState.CLOSED;

    public lowGripper(HardwareMap hardwareMap) {
        lowGripperR = hardwareMap.get(Servo.class, ConstantNamesHardwaremap.LOWGRIPPERRIRGHT);
        lowGripperL = hardwareMap.get(Servo.class, ConstantNamesHardwaremap.LOWGRIPPERLEFT);

        lowGripperR.setDirection(Servo.Direction.REVERSE);
        lowGripperL.setDirection(Servo.Direction.FORWARD);

        setPosition(currentState); // Initialize at default position
    }

    public void toggleGripper() {
        if (currentState == GripperState.CLOSED) {
            setGripperState(GripperState.OPEN);
        } else {
            setGripperState(GripperState.CLOSED);
        }
    }

    public void lowGripperControl(Gamepad gamepad){
        if (gamepad.x && !gripperLock) {
            toggleGripper();
            gripperLock = true;
        } else if (!gamepad.x) {
            gripperLock = false;
        }
    }
    public void setGripperState(GripperState newState) {
        if (newState == GripperState.OPEN) {
            lowGripperR.setPosition(Constants.LOW_GRIPPER_OPENED);
            lowGripperL.setPosition(Constants.LOW_GRIPPER_OPENED);
        } else {
            lowGripperR.setPosition(Constants.LOW_GRIPPER_CLOSED);
            lowGripperL.setPosition(Constants.LOW_GRIPPER_CLOSED);
        }
        currentState = newState;
    }
    public void setPosition(GripperState state) {
        this.currentState = state;
        lowGripperR.setPosition(state.position);
        lowGripperL.setPosition(state.position);
    }

    public GripperState getCurrentState() {
        return currentState;
    }
}
