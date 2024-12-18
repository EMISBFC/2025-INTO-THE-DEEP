package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Gripper SPINA", group = "TeleOp")
public class GripperSpinnerTest extends LinearOpMode {

    private CRServo leftServo;
    private CRServo rightServo;

    // Adjusted position durations (in milliseconds)
    private final int TIME_TO_FACE_DOWN = 1250;  // Longer time to reach 90 degrees down
    private final int TIME_TO_120_UP = 650;      // Shorter time to reach 120 degrees up

    // Logical positions
    private enum Position {
        FACING_UP,   // 90 degrees up (neutral)
        FACING_DOWN, // 90 degrees down
        UP_120       // 120 degrees up
    }

    private Position currentPosition = Position.FACING_UP; // Start at facing up (neutral)

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");

        // Set servo directions
        leftServo.setDirection(CRServo.Direction.FORWARD);
        rightServo.setDirection(CRServo.Direction.REVERSE);

        // Ensure both servos are stopped during initialization
        stopServos();

        telemetry.addData("Status", "Initialized at Neutral (90 degrees up)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.circle) {
                moveToPosition(Position.FACING_DOWN); // Move to 90 degrees down
            } else if (gamepad1.cross) {
                moveToPosition(Position.UP_120); // Move to 120 degrees up
            } else if (gamepad1.triangle) {
                moveToPosition(Position.FACING_UP); // Return to neutral (90 degrees up)
            } else {
                stopServos();
            }

            telemetry.addData("Current Position", currentPosition);
            telemetry.update();
        }
    }

    /**
     * Moves the servos to the desired position by calculating the direction and duration.
     * @param targetPosition The desired target position.
     */
    private void moveToPosition(Position targetPosition) {
        if (currentPosition == targetPosition) {
            telemetry.addData("Action", "Already at " + targetPosition);
            return;
        }

        int duration = calculateMovementDuration(currentPosition, targetPosition)/2;
        double power = calculateMovementDirection(currentPosition, targetPosition);

        telemetry.addData("Action", "Moving to " + targetPosition);
        telemetry.addData("Duration", duration);
        telemetry.addData("Power", power);
        telemetry.update();

        setServoPower(power);
        sleepForDuration(duration);
        stopServos();

        currentPosition = targetPosition; // Update current position
    }

    /**
     * Calculates the movement duration needed to transition between positions.
     * @param from The current position.
     * @param to The target position.
     * @return The time in milliseconds.
     */
    private int calculateMovementDuration(Position from, Position to) {
        if (from == Position.FACING_UP) {
            if (to == Position.FACING_DOWN) return TIME_TO_FACE_DOWN;
            if (to == Position.UP_120) return TIME_TO_120_UP;
        } else if (from == Position.FACING_DOWN) {
            if (to == Position.FACING_UP) return TIME_TO_FACE_DOWN;
            if (to == Position.UP_120) return TIME_TO_FACE_DOWN + TIME_TO_120_UP; // Combined time
        } else if (from == Position.UP_120) {
            if (to == Position.FACING_UP) return TIME_TO_120_UP;
            if (to == Position.FACING_DOWN) return TIME_TO_120_UP + TIME_TO_FACE_DOWN; // Combined time
        }
        return 0; // No movement needed
    }

    /**
     * Calculates the movement direction (positive for up, negative for down).
     * @param from The current position.
     * @param to The target position.
     * @return The power direction (-1.0 to 1.0).
     */
    private double calculateMovementDirection(Position from, Position to) {
        if (from == Position.FACING_UP && to == Position.FACING_DOWN) return -1; // Downward
        if (from == Position.FACING_DOWN && to == Position.FACING_UP) return 1;  // Upward
        if (from == Position.UP_120 && to == Position.FACING_UP) return -1;      // Downward
        if (from == Position.FACING_UP && to == Position.UP_120) return 1;       // Upward
        if (from == Position.FACING_DOWN && to == Position.UP_120) return 1;     // Upward
        if (from == Position.UP_120 && to == Position.FACING_DOWN) return -1;    // Downward
        return 0; // No movement
    }

    /**
     * Sets the power for both servos.
     * @param power Power level (-1.0 to 1.0).
     */
    private void setServoPower(double power) {
        leftServo.setPower(power);
        rightServo.setPower(power);
    }

    /**
     * Stops both servos.
     */
    private void stopServos() {
        leftServo.setPower(0);
        rightServo.setPower(0);
    }

    /**
     * Sleep for the specified duration while allowing telemetry updates.
     * @param duration Time in milliseconds to sleep.
     */
    private void sleepForDuration(int duration) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < duration) {
            idle(); // Allow telemetry updates during sleep
        }
    }
}
