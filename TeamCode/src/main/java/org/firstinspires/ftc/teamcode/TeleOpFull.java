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

@Config
@TeleOp(name="Teleop FULL")
public class TeleOpFull  extends OpMode {
    private RevIMU imu;
    private Chassis chassis;
//    private PIDController controller;
    private lowGripper low_gripper;
    private highGripper high_gripper;
    private GripperSpinner gripperSpinner;
    private Horz horz;
    private Arm arm;


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
        low_gripper = new lowGripper(hardwareMap);
        high_gripper = new highGripper(hardwareMap);
        gripperSpinner = new GripperSpinner(hardwareMap);
        horz = new Horz(hardwareMap); // rename horse variable plz and thank you -mariya
        arm = new Arm(hardwareMap);
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


        chassis.fieldCentricDrive(x, y, rx, heading, acc);
        low_gripper.handleServo(gamepad2);
        high_gripper.handleServo(gamepad2);
        gripperSpinner.handleSpinner(gamepad2);
        horz.handleHorz(gamepad2);
        arm.handleArm(gamepad2);
 //       telemetry.update();

        telemetry.addData("arm pos", arm.currentPos);
        telemetry.addData("arm target", arm.targetArm);
        telemetry.addData("arm power", arm.power);
        telemetry.update();

//        telemetry.addData("horz pos", horz.horz.getCurrentPosition());
//        telemetry.update();
    }
}