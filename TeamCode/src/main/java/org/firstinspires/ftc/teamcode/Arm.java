package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    // Target positions in ticks
    public static final int TRANSITION_POSITION = (int) (30 * Constants.TICKS_IN_DEG);
    public static final int HANG_POSITION = (int) (70 * Constants.TICKS_IN_DEG);
    public static final int GRAB_POSITION = (int) (130 * Constants.TICKS_IN_DEG);

    public int targetArm;
    public double power;
    private final PIDController controller;
    private final DcMotor arm;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(Constants.armP, Constants.armI, Constants.armD);
        targetArm = 0; // Default to initial position
    }

    public void handleArm(Gamepad gamepad) {
        // Check for inputs to transition between positions
        if (gamepad.circle) {
            moveToTransition();
        } else if (gamepad.square) {
            moveToHang();
        } else if (gamepad.cross) {
            moveToGrab();
        }

        // Update motor power with PID
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

    public int currentPos;

    private void updateArm() {
        int currentPos = arm.getCurrentPosition();
        double pid = controller.calculate(currentPos, targetArm);
        double ff = Math.cos(Math.toRadians(currentPos / Constants.TICKS_IN_DEG)) * Constants.armF;
        power = 0.75 * (pid + ff);

        // Clamp power to motor range
        power = Math.max(-1.0, Math.min(1.0, power));

        arm.setPower(power);
    }



}
