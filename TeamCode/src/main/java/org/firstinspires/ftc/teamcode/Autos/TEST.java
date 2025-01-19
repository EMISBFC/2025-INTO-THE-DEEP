package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.NotNeededCantDelete.MecanumDrive;

@Autonomous(name = "TEST", group = "Autonomous")
public class TEST extends LinearOpMode {
    public class AutoArm {

        // Constants for target positions in ticks
        public static final int TRANSITION_POSITION = (int) (30 * Constants.TICKS_IN_DEG);
        public static final int HANG_POSITION = (int) (70 * Constants.TICKS_IN_DEG);
        public static final int GRAB_POSITION = (int) (130 * Constants.TICKS_IN_DEG);

        private final DcMotor armMotor;
        private final PIDController pidController;
        private int targetPosition;

        public AutoArm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotor.class, "arm");
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            pidController = new PIDController(Constants.armP, Constants.armI, Constants.armD);
            targetPosition = 0; // Default starting position
        }

        private void updateArmPosition() {
            int currentPosition = armMotor.getCurrentPosition();
            double pid = pidController.calculate(currentPosition, targetPosition);
            double ff = Math.cos(Math.toRadians(currentPosition / Constants.TICKS_IN_DEG)) * Constants.armF;
            double power = 0.75 * (pid + ff);

            // Clamp power to the motor's range
            power = Math.max(-1.0, Math.min(1.0, power));

            armMotor.setPower(power);
        }

        // Nested classes for actions
        public class ToTransition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = TRANSITION_POSITION;
                updateArmPosition();
                return Math.abs(armMotor.getCurrentPosition() - targetPosition) < Constants.ARMTOLERANCE;
            }
        }

        public class ToHang implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = HANG_POSITION;
                updateArmPosition();
                return Math.abs(armMotor.getCurrentPosition() - targetPosition) < Constants.ARMTOLERANCE;
            }
        }

        public class ToGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = GRAB_POSITION;
                updateArmPosition();
                return Math.abs(armMotor.getCurrentPosition() - targetPosition) < Constants.ARMTOLERANCE;
            }
        }

        // Methods to return actions
        public Action toTransition() {
            return new ToTransition();
        }

        public Action toHang() {
            return new ToHang();
        }

        public Action toGrab() {
            return new ToGrab();
        }
    }

    public class AutoHighGripper {
        private final Servo highGripper;
        private boolean isOpen;

        public AutoHighGripper(HardwareMap hardwareMap) {
            highGripper = hardwareMap.servo.get("highGripperServo");
            highGripper.setPosition(Constants.HIGRIPPER_CLOSE_POS);
            isOpen = false; // Initially closed
        }

        // Action to open the gripper
        public class OpenHighGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                highGripper.setPosition(Constants.HIGRIPPER_OPEN_POS);
                isOpen = true;
                return false;
            }
        }

        // Action to close the gripper
        public class CloseHighGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                highGripper.setPosition(Constants.HIGRIPPER_CLOSE_POS);
                isOpen = false;
                return false;
            }
        }

        // Public methods to return actions
        public Action openGripper() {
            return new OpenHighGripper();
        }

        public Action closeGripper() {
            return new CloseHighGripper();
        }

        // Optional: Getter to check the current state
        public boolean isGripperOpen() {
            return isOpen;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initial robot pose
        Pose2d beginPose = new Pose2d(-4, -72, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        AutoArm arm = new AutoArm(hardwareMap);
        AutoHighGripper highGripper = new AutoHighGripper(hardwareMap);

        // Define the actions with proper starting poses
        Action firstHang = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(-4, -35), Math.toRadians(90.00))
                .build();

        Pose2d firstHangPose = new Pose2d(-4, -35, Math.toRadians(90.00));
        Action secongHang = drive.actionBuilder(firstHangPose)
                .splineTo(new Vector2d(-10, -40), Math.toRadians(90.00))
                .build();

        Pose2d secongHangPose = new Pose2d(-10, -40, Math.toRadians(90.00));
        Action thirdHang = drive.actionBuilder(secongHangPose)
                .splineTo(new Vector2d(-20, -45), Math.toRadians(90.00))
                .build();

        Pose2d thirdHangPose = new Pose2d(-20, -45, Math.toRadians(90.00));
        Action moveSampleAndGrab = drive.actionBuilder(thirdHangPose)
                .splineToLinearHeading(new Pose2d(42, -24, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(62, -30, Math.toRadians(90.00)), Math.toRadians(-90.00))
                .lineToY(-60.48)
                .strafeTo(new Vector2d(62, -56))
                .strafeTo(new Vector2d(45, -70))
                .build();

        Pose2d grabPose = new Pose2d(45, -70, Math.toRadians(90.00));
        Action Grab = drive.actionBuilder(grabPose)
                .strafeTo(new Vector2d(45, -70))
                .build();

        Pose2d parkPose = new Pose2d(45, -70, Math.toRadians(90.00));
        Action park = drive.actionBuilder(parkPose)
                .strafeTo(new Vector2d(42, -70))
                .build();

        waitForStart();

        if (opModeIsActive()) {
            // Execute the trajectory with updated poses
            Actions.runBlocking(new SequentialAction(
                    firstHang,
                    highGripper.openGripper(),
                    moveSampleAndGrab,
                    highGripper.closeGripper(),
                    arm.toTransition(),
                    secongHang,
                    arm.toHang(),
                    highGripper.openGripper(),
                    arm.toGrab(),
                    Grab,
                    highGripper.closeGripper(),
                    arm.toTransition(),
                    thirdHang,
                    arm.toHang(),
                    highGripper.openGripper(),
                    park
            ));
        }
    }


}
