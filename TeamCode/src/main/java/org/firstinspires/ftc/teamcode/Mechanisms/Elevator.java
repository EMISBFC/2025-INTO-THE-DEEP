package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;

public class Elevator {

    // Target positions in ticks
    public static final int BOTTOM_POSITION = 0;
    public static final int MIDDLE_POSITION = 1000; // Replace with actual tick values
    public static final int TOP_POSITION = 2000;   // Replace with actual tick values

    private final DcMotor leftElevatorMotor;
    private final DcMotor rightElevatorMotor;

    private int targetPosition;
    private double power;

    public Elevator(HardwareMap hardwareMap) {
        // Initialize the motors
        leftElevatorMotor = hardwareMap.get(DcMotor.class, ConstantNamesHardwaremap.ELEVATORLEFT);
        rightElevatorMotor = hardwareMap.get(DcMotor.class, ConstantNamesHardwaremap.ELEVATORRIGHT);

        // Reset and set motor modes
        leftElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse one motor to ensure they move in sync
        leftElevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        targetPosition = BOTTOM_POSITION; // Default to the bottom position
    }

    public void handleElevator(Gamepad gamepad) {
        // Use triggers to control elevator
        if (gamepad.right_trigger > 0.1) {
            moveUp(gamepad.right_trigger);
        } else if (gamepad.left_trigger > 0.1) {
            moveDown(gamepad.left_trigger);
        } else {
            stopElevator();
        }

        // Update motor power
        updateElevator();
    }

    public void moveUp(double triggerValue) {
        power = triggerValue; // Scale power
    }

    public void moveDown(double triggerValue) {
        power = -triggerValue; // Scale power
    }

    public void stopElevator() {
        power = 0;
    }

    private void updateElevator() {
        // Set power to both motors
        leftElevatorMotor.setPower(power);
        rightElevatorMotor.setPower(power);
    }
}
