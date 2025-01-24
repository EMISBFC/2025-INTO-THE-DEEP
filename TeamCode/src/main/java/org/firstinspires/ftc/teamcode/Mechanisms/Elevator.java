package org.firstinspires.ftc.teamcode.Mechanisms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;

public class Elevator {

    // Target positions in ticks
    public static int BOTTOM_POSITION = 115;
    public static final int MIDDLE_POSITION = 2000; // Replace with actual tick values
    public static final int TOP_POSITION = 2800;   // Replace with actual tick values

    private final PIDController controller;
    private final DcMotor leftElevatorMotor;
    private final DcMotor rightElevatorMotor;

    private int targetPosition;
    public int leftmoto;
    public int rightmoto;
    public double powerLeft;
    public double powerRight;

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

        // Initialize PID controller
        controller = new PIDController(Constants.elevatorP, Constants.elevatorI, Constants.elevatorD);
        targetPosition = BOTTOM_POSITION; // Default to bottom position
    }

    public void handleElevator(Gamepad gamepad) {
        // Check for inputs to transition between positions
        if (gamepad.dpad_down) {
            moveToBottom();
        } else if (gamepad.dpad_left) {
            moveToMiddle();
        } else if (gamepad.dpad_up) {
            moveToTop();
        }

        // Update motor power with PID
        updateElevator();
    }

    public void moveToBottom() {
        targetPosition = BOTTOM_POSITION;
    }

    public void moveToMiddle() {
        targetPosition = MIDDLE_POSITION;
    }

    public void moveToTop() {
        targetPosition = TOP_POSITION;
    }

    public void updateElevator() {
        int currentPositionLeft = leftElevatorMotor.getCurrentPosition();
        int currentPositionRight = rightElevatorMotor.getCurrentPosition();
        double pidLeft = controller.calculate(currentPositionLeft, targetPosition);
        double pidRight = controller.calculate(currentPositionRight, targetPosition);
        double ffLeft = Math.cos(Math.toRadians(currentPositionLeft / Constants.TICKS_IN_DEG_ELEVATOR)) * Constants.elevatorF;
        double ffRight = Math.cos(Math.toRadians(currentPositionRight / Constants.TICKS_IN_DEG_ELEVATOR)) * Constants.elevatorF;
        powerLeft = 1 * (pidLeft + ffLeft);
        powerRight = 1 * (pidRight + ffRight);

        // Clamp power to motor range
        powerLeft = Math.max(-1.0, Math.min(1.0, powerLeft));
        powerRight = Math.max(-1.0, Math.min(1.0, powerRight));
        leftmoto = leftElevatorMotor.getCurrentPosition();
        rightmoto = rightElevatorMotor.getCurrentPosition();
        leftElevatorMotor.setPower(powerLeft);
        rightElevatorMotor.setPower(powerRight);
    }
}
