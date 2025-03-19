package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Chassis;
import org.firstinspires.ftc.teamcode.Mechanisms.Elevator;
import org.firstinspires.ftc.teamcode.Mechanisms.GripperSpinner;
import org.firstinspires.ftc.teamcode.Mechanisms.HighGripper;
import org.firstinspires.ftc.teamcode.Mechanisms.lowGripper;
import org.firstinspires.ftc.teamcode.NotNeededCantDelete.MecanumDrive;

@Autonomous(name = "TEST left ", group = "Autonomous")
public class LeftAutoNew extends LinearOpMode {
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
        Arm arm = new Arm(hardwareMap);
        Elevator elevator = new Elevator(hardwareMap);
        HighGripper highGripper = new HighGripper(hardwareMap);
        lowGripper lowGripper = new lowGripper(hardwareMap);
        GripperSpinner gripperSpinner = new GripperSpinner(hardwareMap);

        Thread armThread = new Thread(() -> {
            while (!isStopRequested()) {
                arm.updateArm();
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
