package org.firstinspires.ftc.teamcode.Autos;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.Scalar;



@Autonomous(name = "Red Right Auto", group = "Autonomous")
public class SplineHell extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initial robot pose
        Pose2d beginPose = new Pose2d(-4, -72, Math.toRadians(90.00));

        // Initialize your Mecanum drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose); // Replace with your drive class

        // Build the custom trajectory actions
        Action trajectoryAction = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-4, -72))
                .waitSeconds(0.7)
                .splineTo(new Vector2d(8.94, -38.26), Math.toRadians(90.00))
                .splineTo(new Vector2d(26.64, -35.12), Math.toRadians(90.00))
                .splineTo(new Vector2d(36.97, -20.74), Math.toRadians(90.00))
                .splineTo(new Vector2d(48.03, -9.86), Math.toRadians(90.00))
                .build();


        // Wait for the game to start
        waitForStart();

        if (opModeIsActive()) {
            // Execute the trajectory
            Actions.runBlocking(trajectoryAction);
        }
    }
}






