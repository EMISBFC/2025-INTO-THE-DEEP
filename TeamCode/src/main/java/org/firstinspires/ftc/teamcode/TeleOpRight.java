package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Chassis;
import org.firstinspires.ftc.teamcode.Mechanisms.Elevator;
import org.firstinspires.ftc.teamcode.Mechanisms.GripperSpinner;
import org.firstinspires.ftc.teamcode.Mechanisms.HighGripper;
import org.firstinspires.ftc.teamcode.Mechanisms.Horz;
import org.firstinspires.ftc.teamcode.Mechanisms.lowGripper;


@Config
@TeleOp(name="Teleop RIGHT!!")
public class TeleOpRight extends OpMode {
    private RevIMU imu;
    private Chassis chassis;
//    private PIDController controller;
    private lowGripper low_gripper;
    private HighGripper high_gripper;
    private GripperSpinner gripperSpinner;
    private Horz horz;
    private Arm arm;
    private Elevator elevator;


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
        high_gripper = new HighGripper(hardwareMap);
        gripperSpinner = new GripperSpinner(hardwareMap);
        horz = new Horz(hardwareMap); // rename horse variable plz and thank you -mariya
        arm = new Arm(hardwareMap);
        elevator = new Elevator(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Constants.ELEVATOR_BOTTOM_POSITION = 0; // what the fuck why the fuck huh how does this result in 160 what in fuck holy shit what -ofek
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
        low_gripper.handleServo(gamepad1);
        gripperSpinner.handleSpinnerRight(gamepad1);

        high_gripper.handleServo(gamepad2);
        horz.handleHorz(gamepad2);
        arm.handleArmRightTele(gamepad2);
        elevator.updateElevator();

        telemetry.addData("arm pos", arm.currentPos);
        telemetry.addData("horz pos", horz.horz.getCurrentPosition());
        telemetry.addData("max pos", horz.zero_position + Constants.MAX_HORZ_POS);
        telemetry.addData("arm power", arm.power);
        telemetry.addData("elevator power L", elevator.powerLeft);
        telemetry.addData("elevator power R", elevator.powerRight);
        telemetry.addData("Pressed", horz.clicked);
        telemetry.addData("Elevator Right", elevator.ElevatorRightMotorTele);
        telemetry.addData("Elevator Left", elevator.ElevatorLeftMotorTele);
        telemetry.update();
    }
}