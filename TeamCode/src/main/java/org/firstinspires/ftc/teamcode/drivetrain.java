package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class drivetrain {

    private DcMotor FrontLM = null;
    private DcMotor FrontRM = null;
    private DcMotor BackLM = null;
    private DcMotor BackRM = null;
    public CRServo testing = null;
    public CRServo testing1 = null;
    private final double WheelDiameter = 3.75;
    private final double PULSE_PER_REVOLUTION = 537.7;

    private LinearOpMode opmode = null;

    IMU imu;

    public drivetrain() {

    }

    public void init(LinearOpMode opMode) {

        HardwareMap hwMap;

        opmode = opMode;
        hwMap = opMode.hardwareMap;

        imu = hwMap.get(IMU.class, "imu");

        imu.resetYaw();


        FrontRM = hwMap.dcMotor.get("FrontRM");
        FrontLM = hwMap.dcMotor.get("FrontLM");
        BackRM = hwMap.dcMotor.get("BackRM");
        BackLM = hwMap.dcMotor.get("BackLM");

        testing = hwMap.crservo.get("testing");
        testing1 = hwMap.crservo.get("testing1");

        FrontLM.setDirection(FORWARD);
        FrontRM.setDirection(FORWARD);
        BackLM.setDirection(FORWARD);
        BackRM.setDirection(FORWARD);

        FrontRM.setPower(0);
        FrontLM.setPower(0);
        BackLM.setPower(0);
        BackRM.setPower(0);

    }

    public void forward(double speed) {
        FrontRM.setPower(speed);
        FrontLM.setPower(speed);
        BackRM.setPower(speed);
        BackLM.setPower(speed);
    }

    public void backward(double speed) {
        FrontRM.setPower(-speed);
        FrontLM.setPower(-speed);
        BackLM.setPower(-speed);
        BackRM.setPower(-speed);
    }

    public void turnRight(double speed) {
        FrontLM.setPower(speed);
        FrontRM.setPower(-speed);
        BackLM.setPower(speed);
        BackRM.setPower(-speed);
    }

    public void turnLeft(double speed) {
        FrontLM.setPower(-speed);
        FrontRM.setPower(speed);
        BackLM.setPower(-speed);
        BackRM.setPower(speed);
    }

    public void strafeRight(double speed) {
        FrontRM.setPower(-speed);
        FrontLM.setPower(-speed);
        BackRM.setPower(speed);
        BackLM.setPower(speed);
    }

    public void strafeLeft(double speed) {
        FrontRM.setPower(speed);
        FrontLM.setPower(speed);
        BackLM.setPower(-speed);
        BackRM.setPower(-speed);
    }

    public void DriveDistance(double speed, int distance) {
        imu.resetYaw();
        resetEncoders();
        int pulses = calculatePulses(distance);
        FrontLM.setTargetPosition(pulses);
        FrontRM.setTargetPosition(pulses);
        BackLM.setTargetPosition(pulses);
        BackRM.setTargetPosition(pulses);
    }

    public void resetEncoders() {
        BackLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private int calculatePulses(int distance) {
        double circumference = Math.PI * WheelDiameter;
        double rotations = distance / circumference;
        int pulses = (int) (rotations * PULSE_PER_REVOLUTION);
        return pulses;
    }
}