package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.teamcode.Constants.Constants.ELEVATOR_BOTTOM_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.Constants.ELEVATOR_BOTTOM_POSITION_LEFT;
import static org.firstinspires.ftc.teamcode.Constants.Constants.ELEVATOR_TOP_POSITION;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autos.LeftAuto;
import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;

public class Elevator {

    // Target positions in ticks
    private final PIDController controller;
    private final DcMotor leftElevatorMotor;
    private final DcMotor rightElevatorMotor;
    private static int targetLeft;
    private static int targetRight;
    public int ElevatorLeftMotorTele, ElevatorRightMotorTele;
    public double powerLeft, powerRight;
    public static boolean block = false;
    public static final int ELEVATOR_TOLERANCE = 20;

    public Elevator(HardwareMap hardwareMap) {
        // Initialize the motors
        leftElevatorMotor = hardwareMap.get(DcMotor.class, ConstantNamesHardwaremap.ELEVATORLEFT);
        rightElevatorMotor = hardwareMap.get(DcMotor.class, ConstantNamesHardwaremap.ELEVATORRIGHT);

        // Reset and set motor modes
        //leftElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;

        leftElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse one motor to ensure they move in sync
        leftElevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize PID controller
        controller = new PIDController(Constants.elevatorP, Constants.elevatorI, Constants.elevatorD);
        targetLeft = ELEVATOR_BOTTOM_POSITION; // Default to bottom position
        targetRight = ELEVATOR_BOTTOM_POSITION; // Default to bottom position
    }
    public int getCurrentPosLeft(){
        return leftElevatorMotor.getCurrentPosition();
    }

    public int getCurrentPosRight(){
        return rightElevatorMotor.getCurrentPosition();
    }

    public void handleElevator(Gamepad gamepad) {
        // Check for inputs to transition between positions
        if (gamepad.dpad_down) {
            Arm.targetArm = Constants.ARM_TRANSITION_POSITION;
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
        targetRight = ELEVATOR_BOTTOM_POSITION;
        targetLeft = ELEVATOR_BOTTOM_POSITION;
    }

    public void moveToMiddle() {
        targetRight = Constants.ELEVATOR_MIDDLE_POSITION;
        targetLeft = Constants.ELEVATOR_MIDDLE_POSITION + 80;
    }

    public void moveToTop() {
        targetRight = ELEVATOR_TOP_POSITION;
        targetLeft = ELEVATOR_TOP_POSITION + 175;
    }

    public void updateElevator() {
        int currentPositionLeft = leftElevatorMotor.getCurrentPosition();
        int currentPositionRight = rightElevatorMotor.getCurrentPosition();
        double pidLeft = controller.calculate(currentPositionLeft, targetLeft);
        double pidRight = controller.calculate(currentPositionRight, targetRight);
        double ffLeft = Math.cos(Math.toRadians(currentPositionLeft / Constants.TICKS_IN_DEG_ELEVATOR)) * Constants.elevatorF;
        double ffRight = Math.cos(Math.toRadians(currentPositionRight / Constants.TICKS_IN_DEG_ELEVATOR)) * Constants.elevatorF;
        powerLeft = 1 * (pidLeft + ffLeft);
        powerRight = 1 * (pidRight + ffRight);

        // Clamp power to motor range
        powerLeft = Math.max(-1.0, Math.min(1.0, powerLeft));
        powerRight = Math.max(-1.0, Math.min(1.0, powerRight));
        ElevatorLeftMotorTele = leftElevatorMotor.getCurrentPosition();
        ElevatorRightMotorTele = rightElevatorMotor.getCurrentPosition();
        leftElevatorMotor.setPower(powerLeft);
        rightElevatorMotor.setPower(powerRight);
    }

    // Actions
    public class ToTop implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            targetRight = ELEVATOR_TOP_POSITION;
            targetLeft = ELEVATOR_TOP_POSITION + 175;
            return Math.abs(leftElevatorMotor.getCurrentPosition() - targetLeft) < ELEVATOR_TOLERANCE &&
                    Math.abs(rightElevatorMotor.getCurrentPosition() - targetRight) < ELEVATOR_TOLERANCE;
        }
    }

    public  class ToBottom implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            targetRight = ELEVATOR_BOTTOM_POSITION_LEFT;
            targetLeft = ELEVATOR_BOTTOM_POSITION_LEFT;
            return Math.abs(leftElevatorMotor.getCurrentPosition() - targetLeft) < ELEVATOR_TOLERANCE &&
                    Math.abs(rightElevatorMotor.getCurrentPosition() - targetRight) < ELEVATOR_TOLERANCE;
        }
    }

    public void maintainPosition() {
        updateElevator();
    }
    public Action toBottom() {
        return new ToBottom();
    }
    public Action toTop() {
        return new ToTop();
    }
}
