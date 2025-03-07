package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Mechanisms.GripperSpinner.LRot;
import static org.firstinspires.ftc.teamcode.Mechanisms.GripperSpinner.RRot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.Mechanisms.lowGripperTest;


@Config
@TeleOp(name="InTest")
public class IntakeTest extends OpMode {
    private lowGripperTest loGrip;


    @Override
    public void init() {
        loGrip = new lowGripperTest(hardwareMap);
    }

    @Override
    public void loop() {

        loGrip.lowGripperTest(gamepad1);
    }
}