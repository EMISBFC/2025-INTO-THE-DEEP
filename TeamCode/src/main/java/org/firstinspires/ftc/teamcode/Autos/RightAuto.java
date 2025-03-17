package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Chassis;
import org.firstinspires.ftc.teamcode.Mechanisms.Elevator;
import org.firstinspires.ftc.teamcode.Mechanisms.GripperSpinner;
import org.firstinspires.ftc.teamcode.Mechanisms.HighGripper;
import org.firstinspires.ftc.teamcode.NotNeededCantDelete.MecanumDrive;

@Autonomous(name = "Specimen", group = "Autonomous")
public class RightAuto extends LinearOpMode {
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
        public final int ELEVATOR_TOLERANCE = 2; // Adjust based on acceptable range

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

        public class ToTop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = Constants.ELEVATOR_BOTTOM_POSITION;
                updateElevatorPosition();
                return Math.abs(leftElevatorMotor.getCurrentPosition() - targetPosition) < ELEVATOR_TOLERANCE &&
                        Math.abs(rightElevatorMotor.getCurrentPosition() - targetPosition) < ELEVATOR_TOLERANCE;
            }
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

        public Action toTop() {
            return new ToTop();
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
        MecanumDrive drive = new MecanumDrive(hardwareMap,  BlueSpecimenCoordinates.getStart());
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

        MecanumDrive ignitionSystem = new MecanumDrive(hardwareMap, BlueSpecimenCoordinates.getStart());

        Action scorePreLoad = ignitionSystem.actionBuilder(BlueSpecimenCoordinates.getStart())
                .strafeToConstantHeading(BlueSpecimenCoordinates.getScore1().position).build();

        Action moveSpecimens = ignitionSystem.actionBuilder(BlueSpecimenCoordinates.getScore1())
                .splineToConstantHeading(new Vector2d(4, 39), Math.toRadians(-90.00))
                .setTangent(BlueSpecimenCoordinates.getMidWayMoveSpecimensTangent())
                .splineToConstantHeading(BlueSpecimenCoordinates.getBackUp().position, BlueSpecimenCoordinates.getStart().heading)

                .splineToConstantHeading(BlueSpecimenCoordinates.getMoveSpecimensStart0().position, BlueSpecimenCoordinates.getStart().heading)

                .splineToConstantHeading(BlueSpecimenCoordinates.getMoveSpecimenStart1().position, BlueSpecimenCoordinates.getStart().heading)
                .splineToConstantHeading(BlueSpecimenCoordinates.getMoveSpecimenEnd1().position, BlueSpecimenCoordinates.getStart().heading)

                .splineToConstantHeading(BlueSpecimenCoordinates.getMoveSpecimenStart1().position, BlueSpecimenCoordinates.getStart().heading)

                .splineToConstantHeading(BlueSpecimenCoordinates.getMoveSpecimenStart2().position, BlueSpecimenCoordinates.getStart().heading)
                .splineToConstantHeading(BlueSpecimenCoordinates.getMoveSpecimenEnd2().position, BlueSpecimenCoordinates.getStart().heading).build();

        Action collectSecond = ignitionSystem.actionBuilder(BlueSpecimenCoordinates.getMoveSpecimenEnd3())
                .setTangent(BlueSpecimenCoordinates.getStart().heading)
                .splineToConstantHeading(BlueSpecimenCoordinates.getIntakeOne().position, BlueSpecimenCoordinates.getStart().heading).build();

        Action scoreSecond = ignitionSystem.actionBuilder(BlueSpecimenCoordinates.getIntake())
                .setTangent(BlueSpecimenCoordinates.getStart().heading)
                .strafeToConstantHeading(BlueSpecimenCoordinates.getScore2().position).build();

        Action collectThird = ignitionSystem.actionBuilder(BlueSpecimenCoordinates.getScore2())
                .setTangent(BlueSpecimenCoordinates.getStart().heading)
                .strafeToConstantHeading(BlueSpecimenCoordinates.getIntake().position).build();

        Action scoreThird = ignitionSystem.actionBuilder(BlueSpecimenCoordinates.getIntake())
                .setTangent(BlueSpecimenCoordinates.getStart().heading)
                .strafeToConstantHeading(BlueSpecimenCoordinates.getScore3().position).build();

        Action collectFourth = ignitionSystem.actionBuilder(BlueSpecimenCoordinates.getScore3())
                .setTangent(BlueSpecimenCoordinates.getStart().heading)
                .strafeToConstantHeading(BlueSpecimenCoordinates.getIntake().position).build();

        Action scoreFourth = ignitionSystem.actionBuilder(BlueSpecimenCoordinates.getIntake())
                .setTangent(BlueSpecimenCoordinates.getStart().heading)
                .strafeToConstantHeading(BlueSpecimenCoordinates.getScore4().position).build();

        Action park = ignitionSystem.actionBuilder(BlueSpecimenCoordinates.getScore4())
                .setTangent(BlueSpecimenCoordinates.getStart().heading)
                .strafeToConstantHeading(BlueSpecimenCoordinates.getPark().position).build();

        Chassis chassis = new Chassis(hardwareMap);
        chassis.imu.resetYaw();
        waitForStart();



        highGripper.closeGripper();

        waitForStart();

        elevator.toBottom();
        armThread.start();
        elevatorThread.start();

        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            arm.toTransition(),
                            scorePreLoad
                    ),
                    arm.toGrab(),
                    highGripper.openGripper(),
                    moveSpecimens,
                    collectSecond,
                    highGripper.closeGripper(),
                    new ParallelAction(
                            arm.toTransition(),
                            scoreSecond
                    ),
                    arm.toGrab(),
                    highGripper.openGripper(),
                    collectThird,
                    highGripper.closeGripper(),
                    new ParallelAction(
                            arm.toTransition(),
                            scoreThird
                    ),
                    arm.toGrab(),
                    highGripper.openGripper(),
                    collectFourth,
                    highGripper.closeGripper(),
                    new ParallelAction(
                            arm.toTransition(),
                            scoreFourth
                    ),
                    arm.toGrab(),
                    highGripper.openGripper(),
                    park
            ));
        }
        armThread.interrupt();
        armThread.join();
    }
}