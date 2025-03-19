package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Pose2d;

public class Specimen5vCordinates {
    private static final double startX = -30;
    private static final double startY = 63;
    private static final double pushY = 34;
    private static final double firstPushX = -38;
    private static final double secondPushX = -48;
    private static final double thirdPushX = -58;
    private static final double observationGiveY = 50;
    private static final double score1X = 5;
    private static final double score2X = 2.3; //3
    private static final double score3X = 0; //-1.6
    private static final double score4X = -2.7; //-4.2
    private static final double score5X = -5; //-6.2
    private static final double scoreY = 27; //28
    private static final double wallY = 65;
    private static final double intakeX = -38;
    private static final double parkX = -60;
    private static final double startPoseHeading = Math.toRadians(-90);
    private static final double pushHeadingStart = Math.toRadians(-45);
    private static final double pushHeadingEnd = Math.toRadians(45);

    private static final Pose2d start = new Pose2d(startX, startY, startPoseHeading);
    private static final Pose2d firstPushStart = new Pose2d(firstPushX, pushY, pushHeadingStart);
    private static final Pose2d secondPushStart = new Pose2d(secondPushX, pushY, pushHeadingStart);
    private static final Pose2d thirdPushStart = new Pose2d(thirdPushX, pushY, pushHeadingStart);

    private static final Pose2d firstPushEnd = new Pose2d(firstPushX, observationGiveY, pushHeadingEnd);
    private static final Pose2d secondPushEnd = new Pose2d(secondPushX, observationGiveY, pushHeadingEnd);
    private static final Pose2d thirdPushEnd = new Pose2d(thirdPushX, observationGiveY, pushHeadingEnd);

    private static final Pose2d intake = new Pose2d(intakeX, wallY, startPoseHeading);

    private static final Pose2d score1 = new Pose2d(score1X, scoreY, startPoseHeading);
    private static final Pose2d score2 = new Pose2d(score2X, scoreY, startPoseHeading);
    private static final Pose2d score3 = new Pose2d(score3X, scoreY, startPoseHeading);
    private static final Pose2d score4 = new Pose2d(score4X, scoreY, startPoseHeading);
    private static final Pose2d score5 = new Pose2d(score5X, scoreY, startPoseHeading);

    private static final Pose2d park = new Pose2d(parkX, wallY, startPoseHeading);

    public static Pose2d getStart() {
        return start;
    }

    public static Pose2d getMoveSpecimensPush1Start() {
        return firstPushStart;
    }
    public static Pose2d getMoveSpecimensPush2Start() {
        return secondPushStart;
    }
    public static Pose2d getMoveSpecimensPush3Start() {
        return thirdPushStart;
    }

    public static Pose2d getMoveSpecimensPush1End() {
        return firstPushEnd;
    }
    public static Pose2d getMoveSpecimensPush2End() {
        return secondPushEnd;
    }
    public static Pose2d getMoveSpecimensPush3End() {
        return thirdPushEnd;
    }


    public static Pose2d getScore1() {
        return score1;
    }

    public static Pose2d getScore2() {
        return score2;
    }

    public static Pose2d getScore3() {
        return score3;
    }

    public static Pose2d getScore4() {
        return score4;
    }

    public static Pose2d getScore5() {
        return score5;
    }

    public static Pose2d getIntake() {
        return intake;
    }

    public static Pose2d getPark() {
        return park;
    }

}
