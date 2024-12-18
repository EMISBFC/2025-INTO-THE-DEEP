package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class GripperSpinner {

    private CRServo leftServo;
    private CRServo rightServo;

    // Adjusted position durations (in milliseconds)
    private final int faceDown = Constants.TIME_TO_FACE_DOWN;  // Time to reach 90 degrees down
    private final int faceTransition = Constants.TIME_TO_TRANSITION; // Time to reach 120 degrees up
    private GripperSpinner.Position currentPosition = GripperSpinner.Position.FACING_DOWN; // Start at facing down

    private int pause = Constants.GRIPPER_PAUSE;

    // Logical positions
    private enum Position {
        FACING_UP,   // 90 degrees up (neutral)
        FACING_DOWN, // 90 degrees down
        UP_120       // 120 degrees up
    }

    public GripperSpinner(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void handleSpinner(Gamepad gamepad) {
        if (gamepad.triangle) {
            faceTransition();
            sleepForPause();
        } else if (gamepad.dpad_up) {
            faceUp();
            sleepForPause();
        } else if (gamepad.dpad_down) {
            faceDown();
            sleepForPause();
        }
    }

    public void faceDown() {
        if (currentPosition != Position.FACING_DOWN) {
            double power = calculateMovementDirection(currentPosition, Position.FACING_DOWN);
            int duration = calculateMovementDuration(currentPosition, Position.FACING_DOWN);

            setServoPower(power);
            sleepForDuration(duration);
            stopServos();

            currentPosition = Position.FACING_DOWN;
        }
    }

    public void faceUp() {
        if (currentPosition != Position.FACING_UP) {
            double power = calculateMovementDirection(currentPosition, Position.FACING_UP);
            int duration = calculateMovementDuration(currentPosition, Position.FACING_UP);

            setServoPower(power);
            sleepForDuration(duration);
            stopServos();

            currentPosition = Position.FACING_UP;
        }
    }

    public void faceTransition() {
        if (currentPosition != Position.UP_120) {
            double power = calculateMovementDirection(currentPosition, Position.UP_120);
            int duration = calculateMovementDuration(currentPosition, Position.UP_120);

            setServoPower(power);
            sleepForDuration(duration);
            stopServos();

            currentPosition = Position.UP_120;
        }
    }

    private int calculateMovementDuration(GripperSpinner.Position from, GripperSpinner.Position to) {
        if (from == GripperSpinner.Position.FACING_DOWN) {
            if (to == GripperSpinner.Position.FACING_UP) return faceDown; // 90 degrees down -> 90 degrees up
            if (to == GripperSpinner.Position.UP_120) return faceDown+faceTransition; // 90 degrees down -> 120 degrees up
        } else if (from == GripperSpinner.Position.FACING_UP) {
            if (to == GripperSpinner.Position.FACING_DOWN) return faceDown; // 90 degrees up -> 90 degrees down
            if (to == GripperSpinner.Position.UP_120) return faceTransition; // 90 degrees up -> 120 degrees up
        } else if (from == GripperSpinner.Position.UP_120) {
            if (to == GripperSpinner.Position.FACING_UP) return faceTransition; // 120 degrees up -> 90 degrees up
            if (to == GripperSpinner.Position.FACING_DOWN) return faceDown+faceTransition; // 120 degrees up -> 90 degrees down
        }
        return 0; // No movement needed
    }

    private double calculateMovementDirection(GripperSpinner.Position from, GripperSpinner.Position to) {
        if (from == GripperSpinner.Position.FACING_DOWN && to == GripperSpinner.Position.FACING_UP) return 1;  // Upward
        if (from == GripperSpinner.Position.FACING_DOWN && to == GripperSpinner.Position.UP_120) return 1;    // Upward
        if (from == GripperSpinner.Position.FACING_UP && to == GripperSpinner.Position.FACING_DOWN) return -1; // Downward
        if (from == GripperSpinner.Position.FACING_UP && to == GripperSpinner.Position.UP_120) return 1;     // Upward
        if (from == GripperSpinner.Position.UP_120 && to == GripperSpinner.Position.FACING_UP) return -1;    // Downward
        if (from == GripperSpinner.Position.UP_120 && to == GripperSpinner.Position.FACING_DOWN) return -1;  // Downward
        return 0; // No movement
    }

    private void setServoPower(double power) {
        leftServo.setPower(power);
        rightServo.setPower(power);
    }

    private void stopServos() {
        leftServo.setPower(0);
        rightServo.setPower(0);
    }

    private void sleepForDuration(int duration) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < duration) {
            // Waiting
        }
    }

    public void sleepForPause() {
        sleepForDuration(pause);
    }
}
