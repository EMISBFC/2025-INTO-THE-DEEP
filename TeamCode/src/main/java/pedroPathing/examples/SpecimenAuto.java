package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Elevator;
import org.firstinspires.ftc.teamcode.Mechanisms.HighGripper;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "SpecimenAuto", group = "Examples")
public class SpecimenAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState = 0;

    // Mechanism objects (your custom classes)
    private Arm arm;
    private Elevator elevator;
    private HighGripper highGripper;

    private final Pose beginPose = new Pose(9.5, 67.434, Math.toRadians(0));
    private PathBuilder firstHangPath;
    private PathBuilder moveSampleOne;


    public void buildPaths() {
        firstHangPath = new PathBuilder();
        firstHangPath.addPath(
                // Line 1
                new BezierLine(
                        new Point(7.495, 67.175, Point.CARTESIAN),
                        new Point(39.818, 67.175, Point.CARTESIAN)
                )
        ).setConstantHeadingInterpolation(Math.toRadians(0));

        // Build the moveSampleAndGrab chain as a series of segments
        moveSampleOne = new PathBuilder();
        moveSampleOne.addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(39.818, 67.175, Point.CARTESIAN),
                                new Point(4.684, 52.653, Point.CARTESIAN),
                                new Point(40.755, 1.827, Point.CARTESIAN),
                                new Point(116.409, 58.977, Point.CARTESIAN),
                                new Point(67.222, 13.538, Point.CARTESIAN),
                                new Point(18.738, 25.718, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0));
    }

    // Helper to set path state and reset timer
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // Finite state machine updating the autonomous routine
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start: Arm to transition then follow firstHangPath
                arm.moveToHang();
                follower.followPath(firstHangPath.build());
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    // After firstHang: prepare to grab sample and move along moveSampleAndGrabChain
                    arm.moveToGrab();
                    HighGripper.OpenGripper();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(moveSampleOne.build());
                }
                break;
        }
    }

    @Override
    public void init() {
        highGripper = new HighGripper(hardwareMap);
        arm = new Arm(hardwareMap);
        elevator = new Elevator(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // (Optionally, set your custom constants before initializing the follower)
        // Constants.setConstants(FConstants.class, LConstants.class);
        Constants.setConstants(FConstants.class, LConstants.class);
        elevator.updateElevator();
        arm.updateArm();
        // Initialize the follower and set starting pose
        follower = new Follower(hardwareMap);
        follower.setStartingPose(beginPose);

        // Initialize mechanisms
        HighGripper.CloseGripper();

        // Build all paths
        buildPaths();
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        arm.updateArm();
        elevator.updateElevator();
        // Update path following and state machine
        follower.update();
        autonomousPathUpdate();


        follower.telemetryDebug(telemetry);

        // Telemetry for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("elevator", elevator.ElevatorLeftMotorTele);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
