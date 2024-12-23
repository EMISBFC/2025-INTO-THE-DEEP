/* package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Gripper TEST", group = "TeleOp")
public class GripperTest extends LinearOpMode {

    private Servo gripperServo; // Declare the servo
    private final double open = Constants.INTAKE_OPEN_POS; // Adjust based on your hardware setup
    private final double close = Constants.INTAKE_CLOSE_POS ; // Adjust based on your hardware setup

    @Override
    public void runOpMode() {
        // Initialize the hardware
        gripperServo = hardwareMap.get(Servo.class, "gripperServo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Control the gripper using gamepad buttons
            if (gamepad1.circle) {
                gripperServo.setPosition(open); // Open the gripper
                telemetry.addData("Gripper", "Open");
            } else if (gamepad1.square) {
                gripperServo.setPosition(close); // Close the gripper
                telemetry.addData("Gripper", "Close");
            }

            telemetry.update();
        }
    }
} */
