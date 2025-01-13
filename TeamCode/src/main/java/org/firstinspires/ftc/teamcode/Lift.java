package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;

public class Lift {
    private DcMotor liftMotorLeft;
    private DcMotor liftMotorRight;
    private PIDController controller;
    private double power;

    // tuning
    private double kP = 0.01; // Proportional
    private double kI = 0.0;  // Integral
    private double kD = 0.0;  // Derivative

    // pid control variables
    private double lastError = 0;
    private double integral = 0;
    ElapsedTime timer = new ElapsedTime();

    private final int lowPos = 1000;
    private final int midPos = 2000;
    private final int highPos = 3000;


    // Constructor
    public Lift(HardwareMap hardwareMap) {
        liftMotorLeft = hardwareMap.get(DcMotor.class, "liftMotorLeft");
        liftMotorRight = hardwareMap.get(DcMotor.class, "liftMotorRight");

        liftMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        liftMotorRight.setDirection(DcMotor.Direction.REVERSE);
    }

    // Method to calculate PID output
    private double calculatePID(double targetPosition, double currentPosition) {
        double error = targetPosition - currentPosition;
        integral += error;
        double derivative = error - lastError;
        lastError = error;
        integral += (error * timer.seconds());

        return (kP * error) + (kI * integral) + (kD * derivative);
    }

    public void moveToHeight(int targetPosition, Gamepad gamepad) {
        double leftCurrentPosition = liftMotorLeft.getCurrentPosition();
        controller.setPID(kP, kI, kD);

        if (gamepad.x){
            while(targetPosition !=lowPos){
                if(targetPosition > lowPos)targetPosition--;
                else targetPosition++;
            }
        }
        if(gamepad.y){
            while(targetPosition != midPos){
                if(targetPosition > midPos)targetPosition--;
                else targetPosition++;
            }
        }
        if(gamepad.a){
            while(targetPosition != highPos){
                if(targetPosition > highPos)targetPosition--;
                else targetPosition++;
            }
        }

        double pid = calculatePID(targetPosition, leftCurrentPosition);
        double power = 1*pid;
        liftMotorLeft.setPower(power);
        liftMotorRight.setPower(power);
    }
}
