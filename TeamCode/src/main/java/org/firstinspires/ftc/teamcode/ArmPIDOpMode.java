package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name = "Arm PID Control", group = "TeleOp")
public class ArmPIDOpMode extends LinearOpMode {

    private ArmControl armControl;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize the ArmControl class
        armControl = new ArmControl(hardwareMap, "arm");

        // Set initial PID coefficients
        armControl.setPIDCoefficients(ArmPIDConfig.kP, ArmPIDConfig.kI, ArmPIDConfig.kD);

        waitForStart();

        while (opModeIsActive()) {
            // Update PID coefficients dynamically from the dashboard
            armControl.setPIDCoefficients(ArmPIDConfig.kP, ArmPIDConfig.kI, ArmPIDConfig.kD);

            // Check for button presses to set arm positions
            if (gamepad1.circle) {
                armControl.setTargetPositionDegrees(ArmPIDConfig.TRANSITION_POSITION_DEGREES);
            } else if (gamepad1.square) {
                armControl.setTargetPositionDegrees(ArmPIDConfig.SPECIMEN_POSITION_DEGREES);
            } else if (gamepad1.triangle) {
                armControl.setTargetPositionDegrees(ArmPIDConfig.BASKET_POSITION_DEGREES);
            } else if (gamepad1.cross) {
                armControl.setTargetPositionDegrees(ArmPIDConfig.PLAYER_POSITION_DEGREES);
            }

            // Update PID control to move to the target position
            armControl.updatePID();

            // Telemetry for debugging
            telemetry.addData("Target Position (ticks)", armControl.getTargetPosition());
            telemetry.addData("Current Position (ticks)", armControl.getCurrentPosition());
            telemetry.addData("Error (ticks)", armControl.getTargetPosition() - armControl.getCurrentPosition());
            telemetry.update();
        }

        // Stop the arm when the OpMode ends
        armControl.stop();
    }
}

