package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
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

@Autonomous(name = "Specimen5v", group = "Autonomous")
public class RightAuto5v extends LinearOpMode {
    public class AutoArm {
        public static final int TRANSITION_POSITION = (int) (40 * Constants.TICKS_IN_DEG);
        public static final int GRAB_POSITION = (int) (169 * Constants.TICKS_IN_DEG); //165

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

    public class AutoSweeper {
        public Servo Sweep;

        public AutoSweeper(HardwareMap hardwareMap) {
            Sweep = hardwareMap.get(Servo.class, ConstantNamesHardwaremap.SWEEPER);
        }

        public class sweeperPush implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Sweep.setPosition(0.55);
                return false;
            }
        }

        public class sweeperStart implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Sweep.setPosition(0.7);
                return false;
            }
        }

        public class sweeperEnd implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Sweep.setPosition(0.95);
                return false;
            }
        }

        public Action sweeperPush() {
            return new sweeperPush();
        }
        public Action sweeperStart() {
            return new sweeperStart();
        }
        public Action sweeperEnd() {
            return new sweeperEnd();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,  BlueSpecimenCoordinates.getStart());
        AutoArm arm = new AutoArm(hardwareMap);
        AutoElevator elevator = new AutoElevator(hardwareMap);
        AutoHighGripper highGripper = new AutoHighGripper(hardwareMap);
        AutoSweeper sweeper = new AutoSweeper(hardwareMap);
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

        Action pushFirstStart = ignitionSystem.actionBuilder(Specimen5vCordinates.getStart())
                .setTangent(Specimen5vCordinates.getMoveSpecimensPush1Start().heading)
                .splineToConstantHeading(Specimen5vCordinates.getMoveSpecimensPush1Start().position, Specimen5vCordinates.getMoveSpecimensPush1Start().heading).build();

        Action pushFirstEnd = ignitionSystem.actionBuilder(Specimen5vCordinates.getMoveSpecimensPush1Start())
                .setTangent(Specimen5vCordinates.getMoveSpecimensPush1End().heading)
                .splineToConstantHeading(Specimen5vCordinates.getMoveSpecimensPush1End().position, Specimen5vCordinates.getMoveSpecimensPush1End().heading).build();

        Action pushSecondStart = ignitionSystem.actionBuilder(Specimen5vCordinates.getMoveSpecimensPush1End())
                .setTangent(Specimen5vCordinates.getMoveSpecimensPush2Start().heading)
                .splineToConstantHeading(Specimen5vCordinates.getMoveSpecimensPush2Start().position, Specimen5vCordinates.getMoveSpecimensPush2Start().heading).build();

        Action pushSecondEnd = ignitionSystem.actionBuilder(Specimen5vCordinates.getMoveSpecimensPush2Start())
                .setTangent(Specimen5vCordinates.getMoveSpecimensPush2End().heading)
                .splineToConstantHeading(Specimen5vCordinates.getMoveSpecimensPush2End().position, Specimen5vCordinates.getMoveSpecimensPush2End().heading).build();

        Action pushThirdStart = ignitionSystem.actionBuilder(Specimen5vCordinates.getMoveSpecimensPush2End())
                .setTangent(Specimen5vCordinates.getMoveSpecimensPush3Start().heading)
                .splineToConstantHeading(Specimen5vCordinates.getMoveSpecimensPush3Start().position, Specimen5vCordinates.getMoveSpecimensPush3Start().heading).build();

        Action pushThirdEnd = ignitionSystem.actionBuilder(Specimen5vCordinates.getMoveSpecimensPush3Start())
                .setTangent(Specimen5vCordinates.getMoveSpecimensPush3End().heading)
                .splineToConstantHeading(Specimen5vCordinates.getMoveSpecimensPush3End().position, Specimen5vCordinates.getMoveSpecimensPush3End().heading).build();

        Action intake1 = ignitionSystem.actionBuilder(Specimen5vCordinates.getMoveSpecimensPush3End())
                .setTangent(Specimen5vCordinates.getIntake().heading)
                .splineToConstantHeading(Specimen5vCordinates.getIntake().position, Specimen5vCordinates.getIntake().heading).build();

        Action score1 = ignitionSystem.actionBuilder(Specimen5vCordinates.getIntake())
                .setTangent(Specimen5vCordinates.getScore1().heading)
                .splineToConstantHeading(Specimen5vCordinates.getScore1().position, Specimen5vCordinates.getScore1().heading).build();

        Action intake2 = ignitionSystem.actionBuilder(Specimen5vCordinates.getScore1())
                .setTangent(Specimen5vCordinates.getIntake().heading)
                .splineToConstantHeading(Specimen5vCordinates.getIntake().position, Specimen5vCordinates.getIntake().heading).build();

        Action score2 = ignitionSystem.actionBuilder(Specimen5vCordinates.getIntake())
                .setTangent(Specimen5vCordinates.getScore2().heading)
                .splineToConstantHeading(Specimen5vCordinates.getScore2().position, Specimen5vCordinates.getScore2().heading).build();

        Action intake3 = ignitionSystem.actionBuilder(Specimen5vCordinates.getScore2())
                .setTangent(Specimen5vCordinates.getIntake().heading)
                .splineToConstantHeading(Specimen5vCordinates.getIntake().position, Specimen5vCordinates.getIntake().heading).build();

        Action score3 = ignitionSystem.actionBuilder(Specimen5vCordinates.getIntake())
                .setTangent(Specimen5vCordinates.getScore3().heading)
                .splineToConstantHeading(Specimen5vCordinates.getScore3().position, Specimen5vCordinates.getScore3().heading).build();

        Action intake4 = ignitionSystem.actionBuilder(Specimen5vCordinates.getScore3())
                .setTangent(Specimen5vCordinates.getIntake().heading)
                .splineToConstantHeading(Specimen5vCordinates.getIntake().position, Specimen5vCordinates.getIntake().heading).build();

        Action score4 = ignitionSystem.actionBuilder(Specimen5vCordinates.getIntake())
                .setTangent(Specimen5vCordinates.getScore4().heading)
                .splineToConstantHeading(Specimen5vCordinates.getScore4().position, Specimen5vCordinates.getScore4().heading).build();

        Action intake5 = ignitionSystem.actionBuilder(Specimen5vCordinates.getScore4())
                .setTangent(Specimen5vCordinates.getIntake().heading)
                .splineToConstantHeading(Specimen5vCordinates.getIntake().position, Specimen5vCordinates.getIntake().heading).build();

        Action score5 = ignitionSystem.actionBuilder(Specimen5vCordinates.getIntake())
                .setTangent(Specimen5vCordinates.getScore5().heading)
                .splineToConstantHeading(Specimen5vCordinates.getScore5().position, Specimen5vCordinates.getScore5().heading).build();

        Action park = ignitionSystem.actionBuilder(Specimen5vCordinates.getScore5())
                .setTangent(Specimen5vCordinates.getPark().heading)
                .splineToConstantHeading(Specimen5vCordinates.getPark().position, Specimen5vCordinates.getPark().heading).build();


        Chassis chassis = new Chassis(hardwareMap);
        chassis.imu.resetYaw();
        highGripper.openGripper();
        sweeper.sweeperStart();
        waitForStart();

        elevator.toBottom();
        armThread.start();
        elevatorThread.start();

        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            arm.toGrab(),
                            pushFirstStart
                    ),
                    sweeper.sweeperPush(),
                    pushFirstEnd,
                    sweeper.sweeperStart(),

                    pushSecondStart,
                    sweeper.sweeperPush(),
                    pushSecondEnd,
                    sweeper.sweeperStart(),

                    pushThirdStart,
                    sweeper.sweeperPush(),
                    pushThirdEnd,
                    sweeper.sweeperEnd(),

                    intake1,
                    highGripper.closeGripper(),
                    new ParallelAction(
                            arm.toTransition(),
                            score1
                    ),
                    arm.toGrab(),
                    highGripper.openGripper(),
                    intake2,
                    highGripper.closeGripper(),
                    new ParallelAction(
                            arm.toTransition(),
                            score2
                    ),
                    arm.toGrab(),
                    highGripper.openGripper(),
                    intake3,
                    highGripper.closeGripper(),
                    new ParallelAction(
                            arm.toTransition(),
                            score3
                    ),
                    arm.toGrab(),
                    highGripper.openGripper(),
                    intake4,
                    highGripper.closeGripper(),
                    new ParallelAction(
                            arm.toTransition(),
                            score4
                    ),
                    arm.toGrab(),
                    highGripper.openGripper(),
                    intake5,
                    highGripper.closeGripper(),
                    new ParallelAction(
                            arm.toTransition(),
                            score5
                    ),
                    arm.toGrab(),
                    highGripper.openGripper(),
                    park
            ));
        }
    }
}
