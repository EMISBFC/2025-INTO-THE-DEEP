package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmControl {

    private DcMotor armMotor;

    // Conversion for encoder ticks to degrees
    private static final double TICKS_PER_DEGREE = 420.0 / 360.0;

    // PID coefficients
    private double kP, kI, kD;

    // PID variables
    private double integral = 0.0;
    private double previousError = 0.0;

    private int targetPosition = 0;

    public ArmControl(HardwareMap hardwareMap, String motorName) {
        armMotor = hardwareMap.get(DcMotor.class, motorName);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPIDCoefficients(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public void updatePID() {
        double currentPosition = armMotor.getCurrentPosition();
        double error = targetPosition - currentPosition;

        integral += error;
        double derivative = error - previousError;

        double output = (kP * error) + (kI * integral) + (kD * derivative);
        output = Math.max(-1.0, Math.min(1.0, output)); // Limit motor power between -1 and 1

        armMotor.setPower(output);

        previousError = error;
    }

    public void setTargetPositionDegrees(double degrees) {
        targetPosition = (int) (degrees * TICKS_PER_DEGREE);
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public int getCurrentPosition() {
        return armMotor.getCurrentPosition();
    }

    public void stop() {
        armMotor.setPower(0);
    }
}
