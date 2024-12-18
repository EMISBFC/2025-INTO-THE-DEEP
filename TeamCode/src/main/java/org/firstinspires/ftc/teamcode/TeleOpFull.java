package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="Teleop FULL")
public class TeleOpFull extends OpMode {

    private RevIMU imu;
    private Chassis chassis;
    private PIDController controller;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(Constants.p, Constants.i, Constants.d);
        imu = new RevIMU(hardwareMap);
        chassis = new Chassis(hardwareMap);
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

    }
}
