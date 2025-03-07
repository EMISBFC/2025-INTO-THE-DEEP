package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Pose2d;

public class BlueSpecimenCoordinates {
    private static final double startX = -8;
    private static final double startY = 63;
    private static final double score1X = 5;
    private static final double score2X = 0;
    private static final double score3X = -4;
    private static final double score4X = -9;
    private static final double scoreY = 28;
    private static final double backUpY = 40;
    private static final double midWayMoveSpecimensY = 65;
    private static final double moveSpecimensStart0X = -38;
    private static final double moveSpecimensStartY = 13;
    private static final double moveSpecimensEndY = 57;
    private static final double specimen1X = -45.5;
    private static final double specimen2X = -56;
    private static final double specimen3X = -63.5;
    private static final double wallFirstY = 62;
    private static final double wallY = 65;
    private static final double intakeX = -38;
    private static final double parkX = -60;

    private static final double midWayMoveSpecimensTangent = Math.toRadians(-90);
    private static final double startPoseHeading = Math.toRadians(-90);

    private static final Pose2d start = new Pose2d(startX, startY, startPoseHeading);

    private static final Pose2d score1 = new Pose2d(score1X, scoreY, startPoseHeading);
    private static final Pose2d score2 = new Pose2d(score2X, scoreY, startPoseHeading);
    private static final Pose2d score3 = new Pose2d(score3X, scoreY, startPoseHeading);
    private static final Pose2d score4 = new Pose2d(score4X, scoreY, startPoseHeading);
    private static final Pose2d backUp = new Pose2d(moveSpecimensStart0X, backUpY, startPoseHeading);

    private static final Pose2d midWayMoveSpecimens = new Pose2d(moveSpecimensStart0X, midWayMoveSpecimensY, startPoseHeading);
    private static final Pose2d moveSpecimensStart0 = new Pose2d(moveSpecimensStart0X, moveSpecimensStartY, startPoseHeading);

    private static final Pose2d moveSpecimenStart1 = new Pose2d(specimen1X, moveSpecimensStartY, startPoseHeading);
    private static final Pose2d moveSpecimenStart2 = new Pose2d(specimen2X, moveSpecimensStartY, startPoseHeading);
    private static final Pose2d moveSpecimenStart3 = new Pose2d(specimen3X, moveSpecimensStartY, startPoseHeading);

    private static final Pose2d moveSpecimenEnd1 = new Pose2d(specimen1X, moveSpecimensEndY, startPoseHeading);
    private static final Pose2d moveSpecimenEnd2 = new Pose2d(specimen2X, moveSpecimensEndY-10, startPoseHeading);
    private static final Pose2d moveSpecimenEnd3 = new Pose2d(specimen3X, moveSpecimensEndY, startPoseHeading);

    private static final Pose2d intakeOne = new Pose2d(intakeX, wallFirstY, startPoseHeading);
    private static final Pose2d intake = new Pose2d(intakeX, wallY, startPoseHeading);

    private static final Pose2d park = new Pose2d(parkX, wallY, startPoseHeading);

    public static Pose2d getStart() {
        return start;
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


    public static Pose2d getMidWayMoveSpecimens() {
        return midWayMoveSpecimens;
    }
    public static Pose2d getBackUp() {
        return backUp;
    }

    public static Pose2d getMoveSpecimensStart0() {
        return moveSpecimensStart0;
    }

    public static Pose2d getMoveSpecimenStart1() {
        return moveSpecimenStart1;
    }

    public static Pose2d getMoveSpecimenStart2() {
        return moveSpecimenStart2;
    }

    public static Pose2d getMoveSpecimenStart3() {
        return moveSpecimenStart3;
    }

    public static Pose2d getMoveSpecimenEnd1() {
        return moveSpecimenEnd1;
    }

    public static Pose2d getMoveSpecimenEnd2() {
        return moveSpecimenEnd2;
    }

    public static Pose2d getMoveSpecimenEnd3() {
        return moveSpecimenEnd3;
    }

    public static Pose2d getIntake() {
        return intake;
    }

    public static Pose2d getIntakeOne() {
        return intakeOne;
    }

    public static Pose2d getPark() {
        return park;
    }

    public static double getMidWayMoveSpecimensTangent() {
        return midWayMoveSpecimensTangent;
    }
}
