package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autos.LeftAuto;
import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;

import java.util.List;

public class lowGripper {
    public static Servo lowGripperR;
    public static Servo lowGripperL;
    private boolean gripperLock = false;

    public enum GripperState {
        OPEN(Constants.LOW_GRIPPER_OPENED),
        CLOSED(Constants.LOW_GRIPPER_CLOSED),
        OPEN_A_BIT(Constants.LOGRIPPER_A_BIT_OPEN_POS);

        public final double position;
        GripperState(double position) {
            this.position = position;
        }
    }

    private static GripperState currentState = GripperState.CLOSED;

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
        if (gamepad.cross && !gripperLock) {
            toggleGripper();
            gripperLock = true;
        } else if (!gamepad.cross) {
            gripperLock = false;
        }
    }
    public static void setGripperState(GripperState newState) {
        if (newState == GripperState.OPEN) {
            lowGripperR.setPosition(Constants.LOW_GRIPPER_OPENED);
            lowGripperL.setPosition(Constants.LOW_GRIPPER_OPENED);
        } else if (newState == GripperState.CLOSED){
            lowGripperR.setPosition(Constants.LOW_GRIPPER_CLOSED);
            lowGripperL.setPosition(Constants.LOW_GRIPPER_CLOSED);
        } else {
            lowGripperR.setPosition(Constants.LOGRIPPER_A_BIT_OPEN_POS);
            lowGripperL.setPosition(Constants.LOGRIPPER_A_BIT_OPEN_POS);
        }
        currentState = newState;
    }
    public void setPosition(GripperState state) {
        this.currentState = state;
        lowGripperR.setPosition(state.position);
        lowGripperL.setPosition(state.position);
    }

    //Autos

    public class OpenLowGripper implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setGripperState(GripperState.OPEN);
            return false;
        }
    }

    public class CloseLowGripper implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setGripperState(GripperState.CLOSED);
            return false;
        }
    }

    public class OpenABit implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setGripperState(GripperState.OPEN_A_BIT);
            return false;
        }
    }

    public Action openGripper() {
        return new OpenLowGripper();
    }
    public Action openABit() {
        return new OpenABit();
    }

    public Action closeGripper() {
        return new CloseLowGripper();
    }
}
