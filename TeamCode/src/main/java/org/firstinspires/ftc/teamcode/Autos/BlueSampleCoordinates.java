package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Pose2d;

public class BlueSampleCoordinates {
    private static final double startX = 40.5;
    private static final double startY = 63.75;
    private static final double scoreX = 60;
    private static final double scoreY = 56.5;
    private static final double intake2X = 49;
    private static final double intake2Y = 32;
    private static final double intake3X = 60;
    private static final double intake4X = 57;
    private static final double intake4Y = 34;
    private static final double park1X = 36;
    private static final double park2X = 26;
    private static final double parkY = 10;
    private static final double startPoseHeading = Math.toRadians(-180);
    private static final double scorePoseHeading = Math.toRadians(-135);
    private static final double intake2PoseHeading = Math.toRadians(-90);
    private static final double parkPoseHeading = Math.toRadians(-90);
    private static final double scoreTangent = Math.toRadians(-135);
    private static final double intake4PoseHeading = Math.toRadians(-90);
    private static final double intake2HeadingChange = Math.toRadians(-90);
    private static final double intake4HeadingChange = Math.toRadians(-90);
    private static final Pose2d start = new Pose2d(startX, startY, startPoseHeading);
    private static final Pose2d score = new Pose2d(scoreX, scoreY, scorePoseHeading);
    private static final Pose2d intake2 = new Pose2d(intake2X, intake2Y, intake2PoseHeading);
    private static final Pose2d intake3 = new Pose2d(intake3X, intake2Y, intake2PoseHeading);
    private static final Pose2d intake4 = new Pose2d(intake4X, intake4Y, intake4PoseHeading);
    private static final Pose2d park1 = new Pose2d(park1X, parkY, parkPoseHeading);
    private static final Pose2d park2 = new Pose2d(park2X, parkY, parkPoseHeading);

    public static Pose2d getStart() {
        return start;
    }

    public static Pose2d getScore() {
        return score;
    }

    public static Pose2d getIntake2() {
        return intake2;
    }

    public static Pose2d getIntake3() {
        return intake3;
    }

    public static Pose2d getIntake4() {
        return intake4;
    }

    public static Pose2d getPark1() {
        return park1;
    }
    public static Pose2d getPark2() {
        return park2;
    }

    public static double getScoreTangent() {
        return scoreTangent;
    }

    public static double getIntake2HeadingChange() {
        return intake2HeadingChange;
    }

    public static double getIntake4HeadingChange() {
        return intake4HeadingChange;
    }
}
