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

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.NotNeededCantDelete.MecanumDrive;

@Autonomous(name = "Specimen", group = "Autonomous")
public class RightAutoLevelImpossible extends LinearOpMode {
    public class AutoArm {

        public static final int TRANSITION_POSITION = (int) (40 * Constants.TICKS_IN_DEG);
        public static final int GRAB_POSITION = (int) (165 * Constants.TICKS_IN_DEG);

        private final DcMotor armMotor;
        private final PIDController pidController;
        private int targetPosition;

        public AutoArm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotor.class, ConstantNamesHardwaremap.ARM);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            pidController = new PIDController(Constants.armP, Constants.armI, Constants.armD);
            targetPosition = 0;
        }

        private void updateArmPosition() {
            int currentPosition = armMotor.getCurrentPosition();
            double pid = pidController.calculate(currentPosition, targetPosition);
            double ff = Math.cos(Math.toRadians(currentPosition / Constants.TICKS_IN_DEG)) * Constants.armF;
            double power = pid + ff;

            power = Math.max(-1.0, Math.min(1.0, power));
            armMotor.setPower(power);
        }

        public void maintainPosition() {
            updateArmPosition();
        }

        public void setTargetPosition(int position) {
            targetPosition = position;
        }

        public class ToTransition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = TRANSITION_POSITION;
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

        public Action toTransition() {
            return new ToTransition();
        }

        public Action toGrab() {
            return new ToGrab();
        }
    }
    public class AutoElevator {

        private final DcMotor leftElevatorMotor;
        private final DcMotor rightElevatorMotor;
        private final PIDController pidController;

        private int targetPosition;
        public static final int ELEVATOR_TOLERANCE = 2; // Adjust based on acceptable range

        public AutoElevator(HardwareMap hardwareMap) {
            // Initialize motors
            leftElevatorMotor = hardwareMap.get(DcMotor.class, ConstantNamesHardwaremap.ELEVATORLEFT);
            rightElevatorMotor = hardwareMap.get(DcMotor.class, ConstantNamesHardwaremap.ELEVATORRIGHT);

            leftElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftElevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Initialize PID Controller
            pidController = new PIDController(Constants.elevatorP, Constants.elevatorI, Constants.elevatorD);

            targetPosition = Constants.ELEVATOR_BOTTOM_POSITION; // Default to the bottom position
        }

        private void updateElevatorPosition() {
            int currentPositionLeft = leftElevatorMotor.getCurrentPosition();
            int currentPositionRight = rightElevatorMotor.getCurrentPosition();

            double pidLeft = pidController.calculate(currentPositionLeft, targetPosition);
            double pidRight = pidController.calculate(currentPositionRight, targetPosition);

            double ffLeft = Math.cos(Math.toRadians(currentPositionLeft / Constants.TICKS_IN_DEG_ELEVATOR)) * Constants.elevatorF;
            double ffRight = Math.cos(Math.toRadians(currentPositionRight / Constants.TICKS_IN_DEG_ELEVATOR)) * Constants.elevatorF;

            double powerLeft = Math.max(-1.0, Math.min(1.0, pidLeft + ffLeft));
            double powerRight = Math.max(-1.0, Math.min(1.0, pidRight + ffRight));

            leftElevatorMotor.setPower(powerLeft);
            rightElevatorMotor.setPower(powerRight);
        }

        public void maintainPosition() {
            updateElevatorPosition();
        }

        public void setTargetPosition(int position) {
            targetPosition = position;
        }

        public class ToBottom implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = Constants.ELEVATOR_BOTTOM_POSITION;
                updateElevatorPosition();
                return Math.abs(leftElevatorMotor.getCurrentPosition() - targetPosition) < ELEVATOR_TOLERANCE &&
                        Math.abs(rightElevatorMotor.getCurrentPosition() - targetPosition) < ELEVATOR_TOLERANCE;
            }
        }

        public Action toBottom() {
            return new ToBottom();
        }
    }

    public class AutoHighGripper {
        private final Servo highGripper;
        private boolean isOpen;

        public AutoHighGripper(HardwareMap hardwareMap) {
            highGripper = hardwareMap.servo.get(ConstantNamesHardwaremap.HIGHGRIPPER);
            highGripper.setPosition(Constants.HIGRIPPER_CLOSE_POS);
            isOpen = false;
        }
        public class OpenHighGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                highGripper.setPosition(Constants.HIGRIPPER_OPEN_POS);
                isOpen = true;
                return false;
            }
        }

        public class CloseHighGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                highGripper.setPosition(Constants.HIGRIPPER_CLOSE_POS);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                isOpen = false;
                return false;
            }
        }

        public Action openGripper() {
            return new OpenHighGripper();
        }

        public Action closeGripper() {
            return new CloseHighGripper();
        }

        public boolean isGripperOpen() {
            return isOpen;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-4, -75, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        AutoArm arm = new AutoArm(hardwareMap);
        AutoElevator elevator = new AutoElevator(hardwareMap);
        AutoHighGripper highGripper = new AutoHighGripper(hardwareMap);

        Thread armThread = new Thread(() -> {
            while (!isStopRequested()) {
                arm.maintainPosition();
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        });

        Thread elevatorThread = new Thread(() -> {
            while (!isStopRequested()) {
                elevator.maintainPosition();
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        });

        Action firstHang = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(-4, -37.5), Math.toRadians(90.00))
                .build();

        Action moveSampleAndGrab = drive.actionBuilder(new Pose2d(-4, -37.5, Math.toRadians(90.00)))
                .lineToY(-42)
                .splineToLinearHeading(new Pose2d(41, -25, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(62.5, -30, Math.toRadians(90.00)), Math.toRadians(-90.00))
                .lineToY(-62)
                .lineToY(-24)
                .splineToLinearHeading(new Pose2d(75, -30, Math.toRadians(90.00)), Math.toRadians(-90.00))
                .splineToLinearHeading(new Pose2d(75, -60, Math.toRadians(90.00)), Math.toRadians(-90.00))
                .splineToLinearHeading(new Pose2d(60, -59, Math.toRadians(90.00)), Math.toRadians(-90.00))
                .strafeTo(new Vector2d(50, -72))
                .build();

        Action secondHang = drive.actionBuilder(new Pose2d(50, -72, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(-4, -36.5))
                .build();
        Action Grab = drive.actionBuilder( new Pose2d(-4, -36.5, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(50, -72))
                .build();
        Action thirdHang = drive.actionBuilder(new Pose2d(50, -72, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(-8, -36.5))
                .build();
        Action GrabAgain = drive.actionBuilder( new Pose2d(-8, -36.5, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(50, -72))
                .build();
        Action fourthHang = drive.actionBuilder(new Pose2d(50, -72, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(8, -36.5))
                .build();
        Action park = drive.actionBuilder( new Pose2d(8, -36.5, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(1000, -500))
                .build();

        highGripper.closeGripper();

        waitForStart();

        elevator.toBottom();
        armThread.start();
        elevatorThread.start();

        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(
                    arm.toTransition(),
                    firstHang,
                    arm.toGrab(),
                    highGripper.openGripper(),
                    moveSampleAndGrab,
                    highGripper.closeGripper(),
                    arm.toTransition(),
                    secondHang,
                    arm.toGrab(),
                    highGripper.openGripper(),
                    Grab,
                    highGripper.closeGripper(),
                    arm.toTransition(),
                    thirdHang,
                    arm.toGrab(),
                    highGripper.openGripper(),
                    GrabAgain,
                    highGripper.closeGripper(),
                    arm.toTransition(),
                    fourthHang,
                    arm.toGrab(),
                    highGripper.openGripper(),
                    park
            ));
        }
        armThread.interrupt();
        armThread.join();
    }
}
