package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Mechanisms.Elevator.MIDDLE_POSITION;
import static org.firstinspires.ftc.teamcode.Mechanisms.Elevator.TOP_POSITION;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Mechanisms.GripperSpinner;
import org.firstinspires.ftc.teamcode.Mechanisms.HighGripper;
import org.firstinspires.ftc.teamcode.Mechanisms.lowGripper;
import org.firstinspires.ftc.teamcode.NotNeededCantDelete.MecanumDrive;

@Autonomous(name = "Basket Auto", group = "Autonomous")
public class LeftAuto extends LinearOpMode {
    public static boolean startedTransition = false;
    public static class AutoArm {

        public final int TRANSITION_POSITION = Constants.TRANSITION_POSITION;
        public static final int PUT_POSITION = 250;

        private final DcMotor armMotor;
        private final PIDController pidController;
        private int targetPosition;

        public enum TransitionState {
            IDLE,
            GRIPPER_TO_MID_FIRST,
            OPEN_HIGH_GRIPPER,
            ARM_TO_TRANSITION,
            LOW_OPEN,
            LOW_CLOSE,
            LOW_GRIPPER_UP,
            CLOSE_HIGH_GRIPPER,
            OPEN_LOW_GRIPPER,
            ARM_TO_PUT,
            GRIPPER_TO_MID,
            LOW_FINAL_CLOSE
        }

        public AutoArm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotor.class, ConstantNamesHardwaremap.ARM);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
            public boolean run(TelemetryPacket packet) {
                targetPosition = TRANSITION_POSITION;
                updateArmPosition();
                packet.put("Target Position", TRANSITION_POSITION);
                packet.put("Current Position", armMotor.getCurrentPosition());
                return Math.abs(armMotor.getCurrentPosition() - targetPosition) < Constants.ARMTOLERANCE;
            }
        }

        public class ToPut implements Action {
            @Override
            public boolean run(TelemetryPacket packet) {
                targetPosition = PUT_POSITION;
                updateArmPosition();
                packet.put("Target Position", PUT_POSITION);
                packet.put("Current Position", armMotor.getCurrentPosition());
                return Math.abs(armMotor.getCurrentPosition() - targetPosition) < Constants.ARMTOLERANCE;
            }
        }
        private TransitionState transitionState = AutoArm.TransitionState.IDLE;
        private long transitionStartTime;
        public class HandleTransition implements Action {

            @Override
            public boolean run(TelemetryPacket packet) {
                if (transitionState == AutoArm.TransitionState.IDLE) {
                    transitionState = AutoArm.TransitionState.GRIPPER_TO_MID_FIRST;
                    transitionStartTime = System.currentTimeMillis();
                }

                // State machine logic
                switch (transitionState) {

                    case GRIPPER_TO_MID_FIRST:
                        GripperSpinner.LRot.setDirection(Servo.Direction.REVERSE);
                        GripperSpinner.LRot.setPosition(Constants.InRotPosMid);
                        GripperSpinner.RRot.setDirection(Servo.Direction.FORWARD);
                        GripperSpinner.RRot.setPosition(Constants.InRotPosMid);
                        if (System.currentTimeMillis() - transitionStartTime > 400) {
                            transitionState = AutoArm.TransitionState.OPEN_HIGH_GRIPPER;
                            transitionStartTime = System.currentTimeMillis();
                        }
                        break;
                    case OPEN_HIGH_GRIPPER:
                        HighGripper.OpenGripper();
                        if (System.currentTimeMillis() - transitionStartTime > 0) {
                            transitionState = AutoArm.TransitionState.LOW_OPEN;
                            transitionStartTime = System.currentTimeMillis();
                        }
                        break;
                    case LOW_OPEN:
                        lowGripper.OpenLowGripper();
                        if (System.currentTimeMillis() - transitionStartTime > 100) {
                            transitionState = AutoArm.TransitionState.LOW_CLOSE;
                            transitionStartTime = System.currentTimeMillis();
                        }
                        break;
                    case LOW_CLOSE:
                        lowGripper.CloseLowGripper();
                        if (System.currentTimeMillis() - transitionStartTime > 70) {
                            transitionState = AutoArm.TransitionState.LOW_GRIPPER_UP;
                            transitionStartTime = System.currentTimeMillis();
                        }
                        break;

                    case LOW_GRIPPER_UP:
                        GripperSpinner.Up();
                        if (System.currentTimeMillis() - transitionStartTime > 500) {
                            transitionState = AutoArm.TransitionState.CLOSE_HIGH_GRIPPER;
                            transitionStartTime = System.currentTimeMillis();
                        }
                        break;

                    case CLOSE_HIGH_GRIPPER:
                        HighGripper.CloseGripper();
                        if (System.currentTimeMillis() - transitionStartTime > 450) {
                            transitionState = AutoArm.TransitionState.OPEN_LOW_GRIPPER;
                            transitionStartTime = System.currentTimeMillis();
                        }
                        break;

                    case OPEN_LOW_GRIPPER:
                        lowGripper.OpenLowGripper();
                        if (System.currentTimeMillis() - transitionStartTime > 300) {
                            transitionState = AutoArm.TransitionState.ARM_TO_PUT;
                            transitionStartTime = System.currentTimeMillis();
                        }
                        break;

                    case ARM_TO_PUT:
                        toPut();
                        if (System.currentTimeMillis() - transitionStartTime > 30) {
                            transitionState = AutoArm.TransitionState.GRIPPER_TO_MID;
                            transitionStartTime = System.currentTimeMillis();
                        }
                        break;

                    case GRIPPER_TO_MID:
                        GripperSpinner.Mid();
                        if (System.currentTimeMillis() - transitionStartTime > 0) {
                            transitionState = AutoArm.TransitionState.LOW_FINAL_CLOSE; // Transition complete
                        }
                        break;
                    case LOW_FINAL_CLOSE:
                        lowGripper.low_gripper.setPosition(Constants.LOGRIPPER_CLOSE_POS);
                        if (System.currentTimeMillis() - transitionStartTime > 0) {
                            transitionState = AutoArm.TransitionState.IDLE;
                            transitionStartTime = System.currentTimeMillis();
                            startedTransition = false;
                        }
                        break;

                    case IDLE:
                        // Do nothing, waiting for the next command
                        break;

                }
                return false;
            }
        }

        public Action toTransition() {
            return new ToTransition();
        }

        public Action toPut() {
            return new ToPut();
        }

        public Action handleTransition() {
            startedTransition = true;
            return new HandleTransition();
        }
    }
    public class AutoElevator {

        private final DcMotor leftElevatorMotor;
        private final DcMotor rightElevatorMotor;
        private final PIDController pidController;

        private int targetPosition;
        public static final int BOTTOM_POSITION = 505;
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

            targetPosition = BOTTOM_POSITION; // Default to the bottom position
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
                targetPosition = BOTTOM_POSITION;
                updateElevatorPosition();
                return Math.abs(leftElevatorMotor.getCurrentPosition() - targetPosition) < ELEVATOR_TOLERANCE &&
                        Math.abs(rightElevatorMotor.getCurrentPosition() - targetPosition) < ELEVATOR_TOLERANCE;
            }
        }
        public class ToMiddle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = MIDDLE_POSITION;
                updateElevatorPosition();
                return Math.abs(leftElevatorMotor.getCurrentPosition() - targetPosition) < ELEVATOR_TOLERANCE &&
                        Math.abs(rightElevatorMotor.getCurrentPosition() - targetPosition) < ELEVATOR_TOLERANCE;
            }
        }
        public class ToTop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = TOP_POSITION;
                updateElevatorPosition();
                return Math.abs(leftElevatorMotor.getCurrentPosition() - targetPosition) < ELEVATOR_TOLERANCE &&
                        Math.abs(rightElevatorMotor.getCurrentPosition() - targetPosition) < ELEVATOR_TOLERANCE;
            }
        }

        public Action toBottom() {
            return new ToBottom();
        }

        public Action toMiddle() {
            return new ToMiddle();
        }

        public Action toTop() {
            return new ToTop();
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

    public static class AutoLowGripper {
        private final Servo lowGripper;

        public AutoLowGripper(HardwareMap hardwareMap) {
            lowGripper = hardwareMap.get(Servo.class, ConstantNamesHardwaremap.LOWGRIPPER);
        }

        public class OpenLowGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                org.firstinspires.ftc.teamcode.Mechanisms.lowGripper.OpenLowGripper();
                return false;
            }
        }

        public class CloseLowGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                org.firstinspires.ftc.teamcode.Mechanisms.lowGripper.CloseLowGripper();
                return false;
            }
        }

        public Action openGripper() {
            return new OpenLowGripper();
        }

        public Action closeGripper() {
            return new CloseLowGripper();
        }

    }



    @Override
    public void runOpMode() throws InterruptedException {
        GripperSpinner gripperSpinner = new GripperSpinner(hardwareMap);
        Pose2d beginPose = new Pose2d(-39.54, -66.80, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        AutoArm arm = new AutoArm(hardwareMap);
        AutoElevator elevator = new AutoElevator(hardwareMap);
        AutoHighGripper highGripper = new AutoHighGripper(hardwareMap);
        AutoLowGripper lowGripper = new AutoLowGripper(hardwareMap);
        org.firstinspires.ftc.teamcode.Mechanisms.lowGripper lowGripper1 = new lowGripper(hardwareMap);
        highGripper.closeGripper();
        org.firstinspires.ftc.teamcode.Mechanisms.lowGripper.OpenLowGripper();
        HighGripper highGripper1 = new HighGripper(hardwareMap);
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

        Thread transitionThread = new Thread(() -> {
            while (startedTransition) {
                arm.handleTransition();
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

        Action firstPut = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(-65, -62, Math.toRadians(45.00)), Math.toRadians(45.00))
                .waitSeconds(0.2) // BRO I DIED BUT YES DO THIS.
                .build();

        Action wait = drive.actionBuilder(beginPose)
                .waitSeconds(1)
                .build();
        Action waitThree = drive.actionBuilder(beginPose)
                .waitSeconds(4)
                .build();

        Action firstTake = drive.actionBuilder(new Pose2d(-65, -62, Math.toRadians(45.00)))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-55.02, -33.28, Math.toRadians(90.00)), Math.toRadians(90.00))
                .build();
        Action SecondPut = drive.actionBuilder(new Pose2d(-55.02, -33.28, Math.toRadians(45.00)))
                .splineToLinearHeading(new Pose2d(-55.56, -59.51, Math.toRadians(45.00)), Math.toRadians(45.00))
                .waitSeconds(1) // BRO I DIED BUT YES DO THIS.
                .build();

        waitForStart();
        armThread.start();

        if (opModeIsActive()) {
            elevator.toBottom();
            elevatorThread.start();
            transitionThread.start();
            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            firstPut,
                            elevator.toTop()
                    ),
                    new ParallelAction(
                            wait,
                            arm.toPut()
                    ),
                    new ParallelAction(
                            wait,
                            highGripper.openGripper()
                    ),
                    firstTake,
                    arm.toTransition(),
                    elevator.toBottom(),
                    lowGripper.closeGripper(),
                    new ParallelAction(
                            waitThree,
                            arm.handleTransition()
                    ),
                    SecondPut
            ));
        }
        armThread.interrupt();
        armThread.join();
    }
}
