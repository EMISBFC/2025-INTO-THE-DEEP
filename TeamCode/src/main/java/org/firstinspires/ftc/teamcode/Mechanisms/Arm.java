package org.firstinspires.ftc.teamcode.Mechanisms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;

public class Arm {

    // Target positions in ticks
    public static final int TRANSITION_POSITION = -10;
    public static final int PUT_POSITION = 256;
    public static final int HANG_POSITION = 70;
    public static final int GRAB_POSITION = 350;

    public int targetArm;
    public double power;
    private final PIDController controller;
    private final DcMotor arm;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotor.class, ConstantNamesHardwaremap.ARM);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(Constants.armP, Constants.armI, Constants.armD);
        targetArm = 0; // Default to initial position
    }

    public void handleArmRightTele(Gamepad gamepad) {
        // Check for inputs to transition between positions
        if (gamepad.left_bumper) {
            moveToHang();
        } else if (gamepad.right_bumper) {
            moveToGrab();
        }

        // Update motor power with PID
        updateArm();
    }

    public void handleArmLeftTele(Gamepad gamepad) {
        // Check for inputs to transition between positions
        if (gamepad.circle) {
            moveToTransition();
        } else if (gamepad.square) {
            moveToHang();
        } else if (gamepad.cross) {
            moveToGrab();
        }
        updateArm();
    }

    public void moveToTransition() {
        targetArm = TRANSITION_POSITION;
    }

    public void moveToHang() {
        targetArm = HANG_POSITION;
    }

    public void moveToGrab() {
        targetArm = GRAB_POSITION;
    }
    public void moveToPut() {
        targetArm = PUT_POSITION;
    }

    public int currentPos;

    private void updateArm() {
        currentPos = arm.getCurrentPosition();
        double pid = controller.calculate(currentPos, targetArm);
        double ff = Math.cos(Math.toRadians(currentPos / Constants.TICKS_IN_DEG)) * Constants.armF;
        power = 0.75 * (pid + ff);

        // Clamp power to motor range
        power = Math.max(-1.0, Math.min(1.0, power));

        arm.setPower(power);
    }

}