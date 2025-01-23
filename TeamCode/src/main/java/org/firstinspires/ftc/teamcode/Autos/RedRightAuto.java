package org.firstinspires.ftc.teamcode.Autos;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NotNeededCantDelete.MecanumDrive;


@Autonomous(name = "Red Right Auto", group = "Autonomous")
public class RedRightAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initial robot pose
        Pose2d beginPose = new Pose2d(-4, -72, Math.toRadians(90.00));

        // Initialize your Mecanum drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose); // Replace with your drive class

        // Build the custom trajectory actions
        Action trajectoryAction = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-4, -37))
                .strafeTo(new Vector2d(-4 , -45.5))
                .strafeTo(new Vector2d(35, -45.5))
                .strafeTo(new Vector2d(35, -18))
                .strafeTo(new Vector2d(45, -18))
                .strafeTo(new Vector2d(45, -62))
                .strafeTo(new Vector2d(45, -52))
                .strafeTo(new Vector2d(30, -70))
                .strafeTo(new Vector2d(-7, -37))
                .strafeTo(new Vector2d(30, -70))
                .strafeTo(new Vector2d(-7, -37))
                .strafeTo(new Vector2d(30, -65))
                .build();

        // Wait for the game to start
        waitForStart();

        if (opModeIsActive()) {
            // Execute the trajectory
            Actions.runBlocking(trajectoryAction);
        }
    }
}

