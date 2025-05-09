package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Chassis;
import org.firstinspires.ftc.teamcode.Mechanisms.Elevator;
import org.firstinspires.ftc.teamcode.Mechanisms.GripperSpinner;
import org.firstinspires.ftc.teamcode.Mechanisms.HighGripper;
import org.firstinspires.ftc.teamcode.Mechanisms.Horz;
import org.firstinspires.ftc.teamcode.Mechanisms.Sweeper;
import org.firstinspires.ftc.teamcode.Mechanisms.lowGripper;

import java.util.List;


@Config
@TeleOp(name="Teleop RIGHT!!")
public class TeleOpRight extends OpMode {
    private Chassis chassis;
//    private PIDController controller;
    private lowGripper low_gripper;
    private HighGripper high_gripper;
    private GripperSpinner gripperSpinner;
    private Horz horz;
    private Sweeper sweeper;
    private Arm arm;
    private Elevator elevator;
    private List<LynxModule> allHubs;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        for(LynxModule hub : allHubs){
            hub.clearBulkCache(); }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        chassis = new Chassis(hardwareMap);
        low_gripper = new lowGripper(hardwareMap);
        high_gripper = new HighGripper(hardwareMap);
        gripperSpinner = new GripperSpinner(hardwareMap);
        horz = new Horz(hardwareMap); // rename horse variable plz and thank you -mariya
        arm = new Arm(hardwareMap);
        sweeper = new Sweeper(hardwareMap);
//        Constants.ELEVATOR_BOTTOM_POSITION = 90;
        elevator = new Elevator(hardwareMap);
        elevator.moveToBottom();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        Constants.ELEVATOR_BOTTOM_POSITION = 0; // what the fuck why the fuck huh how does this result in 160 what in fuck holy shit what -ofek
    }

    @Override
    public void loop() {
        for(LynxModule hub : allHubs){
            hub.clearBulkCache(); }
        double y = (gamepad1.left_stick_y);
        double x = (-gamepad1.left_stick_x);
        double rx = (-gamepad1.right_stick_x);
        boolean bumper = gamepad1.right_bumper;

        chassis.fieldCentricDrive(x, y, rx, bumper);
        low_gripper.lowGripperControl(gamepad1);
        gripperSpinner.handleInput(gamepad1);
        sweeper.handleSweeper(gamepad1);

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
        telemetry.addData("intake servo pos", low_gripper.lowGripperR.getPosition());
        telemetry.update();
    }
}