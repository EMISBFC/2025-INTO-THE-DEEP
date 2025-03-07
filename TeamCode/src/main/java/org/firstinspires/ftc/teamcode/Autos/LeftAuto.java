package org.firstinspires.ftc.teamcode.Autos;


import static org.firstinspires.ftc.teamcode.Constants.Constants.ELEVATOR_BOTTOM_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.Constants.ELEVATOR_BOTTOM_POSITION_LEFT;
import static org.firstinspires.ftc.teamcode.Constants.Constants.ELEVATOR_TOP_POSITION;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Chassis;
import org.firstinspires.ftc.teamcode.Mechanisms.Elevator;
import org.firstinspires.ftc.teamcode.Mechanisms.GripperSpinner;
import org.firstinspires.ftc.teamcode.Mechanisms.HighGripper;
import org.firstinspires.ftc.teamcode.Mechanisms.lowGripper;
import org.firstinspires.ftc.teamcode.NotNeededCantDelete.MecanumDrive;

@Autonomous(name = "Basket Auto", group = "Autonomous")
public class LeftAuto extends LinearOpMode {
    public class AutoArm {
        public final int TRANSITION_POSITION = Constants.ARM_TRANSITION_POSITION;
        public final int PUT_POSITION = Constants.ARM_PUT_POSITION;

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


        public class ToPut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = PUT_POSITION;
                updateArmPosition();
                return Math.abs(armMotor.getCurrentPosition() - targetPosition) < Constants.ARMTOLERANCE;
            }
        }

        public Action toTransition() {
            return new ToTransition();
        }

        public Action toPut() {return new ToPut();}
    }
    public class AutoElevator {
        public final int ELEVATOR_TOLERANCE = 20; // Adjust based on acceptable range

        private final PIDController pidController;
        private final DcMotor leftElevatorMotor, rightElevatorMotor;
        private int targetLeft, targetRight;

        public AutoElevator(HardwareMap hardwareMap) {
            // Initialize the motors
            leftElevatorMotor = hardwareMap.get(DcMotor.class, ConstantNamesHardwaremap.ELEVATORLEFT);
            rightElevatorMotor = hardwareMap.get(DcMotor.class, ConstantNamesHardwaremap.ELEVATORRIGHT);

            // Reset and set motor modes
            leftElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Reverse one motor to ensure they move in sync
            leftElevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            // Initialize PID controller
            pidController = new PIDController(Constants.elevatorP, Constants.elevatorI, Constants.elevatorD);
            targetLeft = ELEVATOR_BOTTOM_POSITION_LEFT; // Default to bottom position
            targetRight = ELEVATOR_BOTTOM_POSITION_LEFT; // Default to bottom position
        }

        public class ToTop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetRight = ELEVATOR_TOP_POSITION;
                targetLeft = ELEVATOR_TOP_POSITION + 175;
                return Math.abs(leftElevatorMotor.getCurrentPosition() - targetLeft) < ELEVATOR_TOLERANCE &&
                        Math.abs(rightElevatorMotor.getCurrentPosition() - targetRight) < ELEVATOR_TOLERANCE;
            }
        }

        private void updateElevatorPosition() {
            int currentPositionLeft = leftElevatorMotor.getCurrentPosition();
            int currentPositionRight = rightElevatorMotor.getCurrentPosition();

            double pidLeft = pidController.calculate(currentPositionLeft, targetLeft);
            double pidRight = pidController.calculate(currentPositionRight, targetRight);

            double ffLeft = Math.cos(Math.toRadians(currentPositionLeft / Constants.TICKS_IN_DEG_ELEVATOR)) * Constants.elevatorF;
            double ffRight = Math.cos(Math.toRadians(currentPositionRight / Constants.TICKS_IN_DEG_ELEVATOR)) * Constants.elevatorF;

            double powerLeft = Math.max(-1.0, Math.min(1.0, pidLeft + ffLeft));
            double powerRight = Math.max(-1.0, Math.min(1.0, pidRight + ffRight));

            leftElevatorMotor.setPower(powerLeft);
            rightElevatorMotor.setPower(powerRight);
        }
        public class ToBottom implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetRight = ELEVATOR_BOTTOM_POSITION_LEFT;
                targetLeft = ELEVATOR_BOTTOM_POSITION_LEFT;
                return Math.abs(leftElevatorMotor.getCurrentPosition() - targetLeft) < ELEVATOR_TOLERANCE &&
                        Math.abs(rightElevatorMotor.getCurrentPosition() - targetRight) < ELEVATOR_TOLERANCE;
            }
        }
        public void maintainPosition() {
            updateElevatorPosition();
        }
        public Action toBottom() {
            return new AutoElevator.ToBottom();
        }
        public Action toTop() {
            return new AutoElevator.ToTop();
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
            return new AutoHighGripper.OpenHighGripper();
        }

        public Action closeGripper() {
            return new AutoHighGripper.CloseHighGripper();
        }

        public boolean isGripperOpen() {
            return isOpen;
        }
    }
    public class AutoLowGripper{
        public Servo low_gripper;
        public AutoLowGripper(HardwareMap hardwareMap) {
            low_gripper = hardwareMap.servo.get(ConstantNamesHardwaremap.LOWGRIPPER);
            low_gripper.setPosition(Constants.LOGRIPPER_CLOSE_POS);
        }

        public class OpenLowGripper implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                low_gripper.setPosition(Constants.LOGRIPPER_OPEN_POS);
                return false;
            }
        }

        public class CloseLowGripper implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                low_gripper.setPosition(Constants.LOGRIPPER_CLOSE_POS);
                return false;
            }
        }

        public class OpenABit implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                low_gripper.setPosition(Constants.LOGRIPPER_A_BIT_OPEN_POS);
                return false;
            }
        }

        public Action openGripper() {
            return new AutoLowGripper.OpenLowGripper();
        }
        public Action openABit() {
            return new AutoLowGripper.OpenABit();
        }

        public Action closeGripper() {
            return new AutoLowGripper.CloseLowGripper();
        }
    }
    public class AutoGripperSpinner {
        public Servo LRot;
        public Servo RRot;

        public AutoGripperSpinner(HardwareMap hardwareMap){
            LRot = hardwareMap.servo.get(ConstantNamesHardwaremap.GRIPPERSPINNERLEFT);
            RRot = hardwareMap.servo.get(ConstantNamesHardwaremap.GRIPPERSPINNERRIGHT);
            LRot.setDirection(Servo.Direction.REVERSE);
            //LRot.setPosition(Constants.InRotPosDown);
            RRot.setDirection(Servo.Direction.FORWARD);
            //RRot.setPosition(Constants.InRotPosDown);
        }
        public class Down implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                LRot.setDirection(Servo.Direction.REVERSE);
                LRot.setPosition(Constants.InRotPosDown);
                RRot.setDirection(Servo.Direction.FORWARD);
                RRot.setPosition(Constants.InRotPosDown);
                return false;
            }
        }

        public class Up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                LRot.setDirection(Servo.Direction.REVERSE);
                LRot.setPosition(Constants.InRotPosUp);
                RRot.setDirection(Servo.Direction.FORWARD);
                RRot.setPosition(Constants.InRotPosUp);
                return false;
            }
        }

        public class Mid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                LRot.setDirection(Servo.Direction.REVERSE);
                LRot.setPosition(Constants.InRotPosMid);
                RRot.setDirection(Servo.Direction.FORWARD);
                RRot.setPosition(Constants.InRotPosMid);
                return false;
            }

        }

        public Action down() {
            return new AutoGripperSpinner.Down();
        }

        public Action mid() {
            return new AutoGripperSpinner.Mid();
        }

        public Action up() {
            return new AutoGripperSpinner.Up();
        }
    }

    public class RESETGOD implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Chassis chassis = new Chassis(hardwareMap);
            chassis.imu.resetYaw();
            return false;
        }
    }

    public Action reset() {
        return new RESETGOD();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        AutoArm arm = new AutoArm(hardwareMap);
        AutoElevator elevator = new AutoElevator(hardwareMap);
        AutoHighGripper highGripper = new AutoHighGripper(hardwareMap);
        AutoLowGripper lowGripper = new AutoLowGripper(hardwareMap);
        AutoGripperSpinner gripperSpinner = new AutoGripperSpinner(hardwareMap);

        Thread armThread = new Thread(() -> {
            while (!isStopRequested()) {
                arm.updateArmPosition();
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

        MecanumDrive ignitionSystem = new MecanumDrive(hardwareMap, BlueSampleCoordinates.getStart());

        Action scorePreLoad = ignitionSystem.actionBuilder(BlueSampleCoordinates.getStart())
                .setTangent(BlueSampleCoordinates.getScoreTangent())
                .splineToLinearHeading(BlueSampleCoordinates.getScore(), BlueSampleCoordinates.getIntake2HeadingChange())
                .build();

        Action intake2 = ignitionSystem.actionBuilder(BlueSampleCoordinates.getScore())
                .setTangent(BlueSampleCoordinates.getScoreTangent())
                .splineToLinearHeading(BlueSampleCoordinates.getIntake2(), BlueSampleCoordinates.getIntake2HeadingChange())
                .build();

        Action score2 = ignitionSystem.actionBuilder(BlueSampleCoordinates.getIntake2())
                .setTangent(BlueSampleCoordinates.getScoreTangent())
                .setTangent(BlueSampleCoordinates.getScoreTangent())
                .splineToLinearHeading(BlueSampleCoordinates.getScore(), BlueSampleCoordinates.getIntake2HeadingChange())
                .build();

        Action intake3 = ignitionSystem.actionBuilder(BlueSampleCoordinates.getScore())
                .setTangent(BlueSampleCoordinates.getScoreTangent())
                .splineToLinearHeading(BlueSampleCoordinates.getIntake3(), BlueSampleCoordinates.getIntake2HeadingChange())
                .build();

        Action score3 = ignitionSystem.actionBuilder(BlueSampleCoordinates.getIntake3())
                .setTangent(BlueSampleCoordinates.getScoreTangent())
                .setTangent(BlueSampleCoordinates.getScoreTangent())
                .splineToLinearHeading(BlueSampleCoordinates.getScore(), BlueSampleCoordinates.getIntake2HeadingChange())
                .build();

        Action intake4 = ignitionSystem.actionBuilder(BlueSampleCoordinates.getScore())
                .setTangent(BlueSampleCoordinates.getScoreTangent())
                .splineToLinearHeading(BlueSampleCoordinates.getIntake4(), BlueSampleCoordinates.getIntake4HeadingChange())
                .build();

        Action score4 = ignitionSystem.actionBuilder(BlueSampleCoordinates.getIntake4())
                .setTangent(BlueSampleCoordinates.getScoreTangent())
                .splineToLinearHeading(BlueSampleCoordinates.getScore(), BlueSampleCoordinates.getIntake4HeadingChange())
                .build();

        Action prePark = ignitionSystem.actionBuilder(BlueSampleCoordinates.getScore())
                .setTangent(BlueSampleCoordinates.getScoreTangent())
                .strafeToLinearHeading(BlueSampleCoordinates.getPark1().component1(), BlueSampleCoordinates.getPark1().heading)
                .build();

        Action park = ignitionSystem.actionBuilder(BlueSampleCoordinates.getPark1())
                .setTangent(0)
                .strafeToLinearHeading(BlueSampleCoordinates.getPark2().component1(), 135)
                .build();

        Action wait = ignitionSystem.actionBuilder(BlueSampleCoordinates.getScore())
                .waitSeconds(4)
                .build();

        waitForStart();

        highGripper.closeGripper();
        lowGripper.openGripper();
        waitForStart();

        armThread.start();
        elevatorThread.start();


        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        elevator.toTop(),
                        scorePreLoad,
                        lowGripper.openGripper(),
                        gripperSpinner.down(),
                        new SleepAction(0.4)
                ),
                arm.toPut(),
                new SleepAction(0.5),
                highGripper.openGripper(),
                new SleepAction(0.7),
                arm.toTransition(),
                new SleepAction(0.2),

                new ParallelAction(
                        elevator.toBottom(),
                        intake2
                ),


                new SleepAction(0.4),
                lowGripper.closeGripper(),
                new SleepAction(0.3),
                gripperSpinner.mid(),
                new SleepAction(0.3),
                lowGripper.openABit(),
                new SleepAction(0.3),
                lowGripper.closeGripper(),
                new SleepAction(0.3),
                gripperSpinner.up(),
                new SleepAction(0.3),
                highGripper.closeGripper(),
                lowGripper.openGripper(),
                new SleepAction(0.3),
                elevator.toTop(),
                gripperSpinner.down(),
                new SleepAction(0.3),

                /////////////////////////////////
                new ParallelAction(
                        elevator.toTop(),
                        score2,
                        lowGripper.openGripper(),
                        gripperSpinner.down(),
                        new SleepAction(0.4)
                ),
                arm.toPut(),
                new SleepAction(0.5),
                highGripper.openGripper(),
                new SleepAction(0.7),
                arm.toTransition(),
                new SleepAction(0.2),

                new ParallelAction(
                        elevator.toBottom(),
                        intake3
                ),
                new SleepAction(0.4),
                lowGripper.closeGripper(),
                new SleepAction(0.3),
                gripperSpinner.mid(),
                new SleepAction(0.3),
                lowGripper.openABit(),
                new SleepAction(0.3),
                lowGripper.closeGripper(),
                new SleepAction(0.3),
                gripperSpinner.up(),
                new SleepAction(0.3),
                highGripper.closeGripper(),
                lowGripper.openGripper(),
                new SleepAction(0.3),
                new ParallelAction(
                        elevator.toTop(),
                        score3,
                        lowGripper.openGripper(),
                        gripperSpinner.down(),
                        new SleepAction(0.4)
                ),

                arm.toPut(),
                new SleepAction(0.5),
                highGripper.openGripper(),
                new SleepAction(0.7),
                arm.toTransition(),
                new SleepAction(0.2),
                elevator.toBottom(),
                prePark,
                new SleepAction(0.3),
                reset(),
                arm.toPut(),
                new SleepAction(0.3),
                park,
                new SleepAction(0.5),
                arm.toTransition(),
                new SleepAction(0.5)
        ));

        armThread.interrupt();
        armThread.join();
    }
}
