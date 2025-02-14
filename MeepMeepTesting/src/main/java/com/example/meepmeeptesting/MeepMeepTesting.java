package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-4, -75, Math.toRadians(90)))                .lineToY(-42)
                .splineToLinearHeading(new Pose2d(42, -24, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(66, -28.5, Math.toRadians(90.00)), Math.toRadians(-90.00))
                .lineToY(-62)
                .lineToY(-24)
                .splineToLinearHeading(new Pose2d(78, -30, Math.toRadians(90.00)), Math.toRadians(-90.00))
                .splineToLinearHeading(new Pose2d(78, -60, Math.toRadians(90.00)), Math.toRadians(-90.00))
                .splineToLinearHeading(new Pose2d(60, -59, Math.toRadians(90.00)), Math.toRadians(-90.00))
                .strafeTo(new Vector2d(50, -72.3))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}