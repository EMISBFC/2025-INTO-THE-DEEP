package org.firstinspires.ftc.teamcode.Mechanisms;


import static org.firstinspires.ftc.teamcode.Mechanisms.Elevator.block;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autos.LeftAuto;
import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;

public class Arm {

    static public int targetArm;
    public double power;
    private final PIDController controller;
    private final DcMotor arm;

    public final int TRANSITION_POSITION = Constants.ARM_TRANSITION_POSITION;
    public final int PUT_POSITION = Constants.ARM_PUT_POSITION;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotor.class, ConstantNamesHardwaremap.ARM);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller = new PIDController(Constants.armP, Constants.armI, Constants.armD);
        targetArm = 0; // go to 0 (i.e. don't move) for initial position
    }

    public void handleArmRightTele(Gamepad gamepad) {
        // Check for inputs to transition between positions
        if (gamepad.left_bumper) {
            moveToHang();
        } else if (gamepad.right_bumper) {
            moveToGrab();
        }
        else if (gamepad.left_stick_button) {
            moveToSafety();
        }

        // Update motor power with PID
        updateArm();
    }

    public void handleArmLeftTele(Gamepad gamepad) {
        // Check for inputs to transition between positions
        if (gamepad.left_bumper) {
            moveToTransition();
        } else if (gamepad.right_bumper) {
            moveToPut();
        }
        updateArm();
    }

    public enum TransitionState {
        IDLE,
        OPEN_HIGH_GRIPPER,
        ARM_TO_TRANSITION,
        LOW_OPEN,
        LOW_CLOSE,
        GRIPPER_TO_MID_FIRST,
        LOW_GRIPPER_UP,
        CLOSE_HIGH_GRIPPER,
        OPEN_LOW_GRIPPER,
        ARM_TO_PUT,
        GRIPPER_TO_MID,
        LOW_FINAL_CLOSE
    }

    private TransitionState transitionState = TransitionState.IDLE;
    private long transitionStartTime;

    public void handleTransition(Gamepad gamepad) {
        // Start transition if dpad_up is pressed and we are in the IDLE state
        if (gamepad.triangle && transitionState == TransitionState.IDLE) {
            transitionState = TransitionState.GRIPPER_TO_MID_FIRST;
            transitionStartTime = System.currentTimeMillis();
        }

        // State machine logic
        switch (transitionState) {

            case GRIPPER_TO_MID_FIRST:
                GripperSpinner.LRot.setDirection(Servo.Direction.FORWARD);
                GripperSpinner.LRot.setPosition(Constants.InRotPosMid);
                GripperSpinner.RRot.setDirection(Servo.Direction.REVERSE);
                GripperSpinner.RRot.setPosition(Constants.InRotPosMid);
                if (System.currentTimeMillis() - transitionStartTime > 400) {
                    transitionState = TransitionState.OPEN_HIGH_GRIPPER;
                    transitionStartTime = System.currentTimeMillis();
                }
                break;
            case OPEN_HIGH_GRIPPER:
                HighGripper.OpenGripper();
                if (System.currentTimeMillis() - transitionStartTime > 100) {
                    transitionState = TransitionState.ARM_TO_TRANSITION;
                    transitionStartTime = System.currentTimeMillis();
                }
                break;

            case ARM_TO_TRANSITION:
                moveToTransition();
                if (System.currentTimeMillis() - transitionStartTime > 300) {
                    transitionState = TransitionState.LOW_OPEN;
                    transitionStartTime = System.currentTimeMillis();
                }
                break;

            case LOW_OPEN:
                lowGripper.setGripperState(lowGripper.GripperState.OPEN_A_BIT);
                //lowGripper.lowGripperL.setPosition(0.75);
                //lowGripper.lowGripperL.setPosition(Constants.LOGRIPPER_A_BIT_OPEN_POS_LEFT);
                if (System.currentTimeMillis() - transitionStartTime > 300) {
                    transitionState = TransitionState.LOW_CLOSE;
                    transitionStartTime = System.currentTimeMillis();
                }
                break;
            case LOW_CLOSE:
                //lowGripper.lowGripperR.setPosition(Constants.LOGRIPPER_CLOSE_POS);
                //lowGripper.lowGripperL.setPosition(Constants.LOGRIPPER_CLOSE_POS);
                lowGripper.setGripperState(lowGripper.GripperState.CLOSED);
                if (System.currentTimeMillis() - transitionStartTime > 300) {
                    transitionState = TransitionState.LOW_GRIPPER_UP;
                    transitionStartTime = System.currentTimeMillis();
                }
                break;

            case LOW_GRIPPER_UP:
                GripperSpinner.LRot.setDirection(Servo.Direction.FORWARD);
                GripperSpinner.LRot.setPosition(Constants.InRotPosUp);
                GripperSpinner.RRot.setDirection(Servo.Direction.REVERSE);
                GripperSpinner.RRot.setPosition(Constants.InRotPosUp);
                if (System.currentTimeMillis() - transitionStartTime > 500) {
                    transitionState = TransitionState.CLOSE_HIGH_GRIPPER;
                    transitionStartTime = System.currentTimeMillis();
                }
                break;

            case CLOSE_HIGH_GRIPPER:
                HighGripper.CloseGripper();
                if (System.currentTimeMillis() - transitionStartTime > 450) {
                    transitionState = TransitionState.OPEN_LOW_GRIPPER;
                    transitionStartTime = System.currentTimeMillis();
                }
                break;

            case OPEN_LOW_GRIPPER:
                lowGripper.setGripperState(lowGripper.GripperState.OPEN);
                //lowGripper.lowGripperR.setPosition(0.75);
                //lowGripper.lowGripperL.setPosition(0.75);
                if (System.currentTimeMillis() - transitionStartTime > 100) {
                    transitionState = TransitionState.ARM_TO_PUT;
                    transitionStartTime = System.currentTimeMillis();
                }
                break;

            case ARM_TO_PUT:
                moveToPut();
                if (System.currentTimeMillis() - transitionStartTime > 30) {
                    transitionState = TransitionState.GRIPPER_TO_MID;
                    transitionStartTime = System.currentTimeMillis();
                }
                break;

            case GRIPPER_TO_MID:
                GripperSpinner.LRot.setDirection(Servo.Direction.FORWARD);
                GripperSpinner.LRot.setPosition(Constants.InRotPosMid);
                GripperSpinner.RRot.setDirection(Servo.Direction.REVERSE);
                GripperSpinner.RRot.setPosition(Constants.InRotPosMid);
                block = true;
                if (System.currentTimeMillis() - transitionStartTime > 0) {
                    transitionState = TransitionState.IDLE; // Transition complete
                }
                break;
            case LOW_FINAL_CLOSE:
                lowGripper.setGripperState(lowGripper.GripperState.OPEN);
//                lowGripper.lowGripperR.setPosition(Constants.LOGRIPPER_CLOSE_POS);
//                lowGripper.lowGripperL.setPosition(Constants.LOGRIPPER_CLOSE_POS);
                if (System.currentTimeMillis() - transitionStartTime > 0) {
                    transitionState = TransitionState.IDLE;
                    transitionStartTime = System.currentTimeMillis();
                }
                break;

            case IDLE:
                // Do nothing, waiting for the next command
                break;
        }
    }

    public void moveToTransition() {
        targetArm = Constants.ARM_TRANSITION_POSITION;
        block = false;
    }

    public void moveToHang() {
        targetArm = Constants.ARM_HANG_POSITION;
    }

    public void moveToGrab() {
        targetArm = Constants.ARM_GRAB_POSITION;
    }
    public void moveToPut() {
        targetArm = Constants.ARM_PUT_POSITION;
        block = true;
    }
    public void moveToSafety() {
        targetArm = Constants.ARM_GRAB_POSITION_SAFETY;
    }

    public int currentPos;

    public int getCurrentPos(){
        return arm.getCurrentPosition();
    }
    public void updateArm() {
        currentPos = arm.getCurrentPosition();
        double pid = controller.calculate(currentPos, targetArm);
        double ff = Math.cos(Math.toRadians(currentPos / Constants.TICKS_IN_DEG)) * Constants.armF;
        power = 0.75 * (pid + ff);

        // Clamp power to motor range
        power = Math.max(-1.0, Math.min(1.0, power));

        arm.setPower(power);
    }

    // Actions
    public void maintainPosition() {
        updateArm();
    }

    public void setTargetPosition(int position) {
        targetArm = position;
    }

    public class ToTransition implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            targetArm = TRANSITION_POSITION;
            updateArm();
            return Math.abs(arm.getCurrentPosition() - targetArm) < Constants.ARMTOLERANCE;
        }
    }


    public class ToPut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            targetArm = PUT_POSITION;
            updateArm();
            return Math.abs(arm.getCurrentPosition() - targetArm) < Constants.ARMTOLERANCE;
        }
    }

    public Action toTransition() {
        return new ToTransition();
    }

    public Action toPut() {return new ToPut();}
}