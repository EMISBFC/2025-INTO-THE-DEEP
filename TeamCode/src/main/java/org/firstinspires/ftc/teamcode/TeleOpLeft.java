package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Mechanisms.GripperSpinner.LRot;
import static org.firstinspires.ftc.teamcode.Mechanisms.GripperSpinner.RRot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Chassis;
import org.firstinspires.ftc.teamcode.Mechanisms.Elevator;
import org.firstinspires.ftc.teamcode.Mechanisms.GripperSpinner;
import org.firstinspires.ftc.teamcode.Mechanisms.HighGripper;
import org.firstinspires.ftc.teamcode.Mechanisms.Horz;
import org.firstinspires.ftc.teamcode.Mechanisms.lowGripper;


@Config
@TeleOp(name="Teleop LEFT!!")
public class TeleOpLeft extends OpMode {
    private Chassis chassis;
    //    private PIDController controller;
    private lowGripper low_gripper;
    private HighGripper high_gripper;
    private GripperSpinner gripperSpinner;
    private Horz horz;
    private Arm arm;
    private Elevator elevator;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        chassis = new Chassis(hardwareMap);
        low_gripper = new lowGripper(hardwareMap);

        high_gripper = new HighGripper(hardwareMap);
        gripperSpinner = new GripperSpinner(hardwareMap);

        LRot.setDirection(Servo.Direction.REVERSE);
        LRot.setPosition(Constants.InRotPosMid);
        RRot.setDirection(Servo.Direction.FORWARD);
        RRot.setPosition(Constants.InRotPosMid);
        horz = new Horz(hardwareMap); // rename horse variable plz and thank you -mariya
        arm = new Arm(hardwareMap);
        Constants.ELEVATOR_BOTTOM_POSITION = Constants.ELEVATOR_BOTTOM_POSITION_LEFT;
        elevator = new Elevator(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        double y = (gamepad1.left_stick_y);
        double x = (-gamepad1.left_stick_x);
        double rx = (-gamepad1.right_stick_x);
        double acc = gamepad1.right_trigger;


        chassis.fieldCentricDrive(x, y, rx, acc);
        low_gripper.handleServo(gamepad1);
        gripperSpinner.handleSpinnerLeft(gamepad1);

        high_gripper.handleServo(gamepad2);
        horz.handleHorz(gamepad2);
        arm.handleArmLeftTele(gamepad2);
        arm.handleTransition(gamepad2);

        elevator.handleElevator(gamepad2);

        telemetry.addData("arm pos", arm.currentPos);
        telemetry.addData("horz pos", horz.horz.getCurrentPosition());
        telemetry.addData("arm power", arm.power);
        telemetry.addData("elevator power L", elevator.powerLeft);
        telemetry.addData("elevator power R", elevator.powerRight);
        telemetry.addData("Elevator Right", elevator.ElevatorRightMotorTele);
        telemetry.addData("Elevator Left", elevator.ElevatorLeftMotorTele);
        telemetry.update();
    }
}