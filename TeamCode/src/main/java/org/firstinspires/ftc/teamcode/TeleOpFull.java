package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU.Parameters;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Config
@TeleOp(name="Teleop FULL")
public class TeleOpFull  extends OpMode {
    private RevIMU imu;

    private Chassis chassis;
    private PIDController controller;
    private GripperLearn gripper;
    private GripperSpinner gripperSpinner;
    private Horz horz;

    public static RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu = new RevIMU(hardwareMap);
        chassis = new Chassis(hardwareMap);
        gripper = new GripperLearn(hardwareMap);
        gripperSpinner = new GripperSpinner(hardwareMap);
        horz = new Horz(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu.init();

    }


    @Override
    public void loop() {
        double y = (gamepad1.left_stick_y);
        double x = (-gamepad1.left_stick_x);
        double rx = (-gamepad1.right_stick_x);
        double acc = gamepad1.right_trigger;
        double heading = imu.getRotation2d().getDegrees();

        telemetry.addData("Encoder Position", horz.horz.getCurrentPosition());
        telemetry.update();
        chassis.fieldCentricDrive(x, y, rx, heading, acc);
        gripper.handleServo(gamepad2);
        gripperSpinner.handleSpinner(gamepad2);
        telemetry.update();
        horz.handleHorz(gamepad2);
    }
}
