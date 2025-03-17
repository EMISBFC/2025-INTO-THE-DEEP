package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static double armP = 0.02, armI = 0.025, armD= 0.001, armF = 0.275;
    public static final double ARMTOLERANCE = 5;
    public static int ARM_TRANSITION_POSITION = -28, ARM_HANG_POSITION = 70, ARM_PUT_POSITION = 250, ARM_GRAB_POSITION = 340, ARM_GRAB_POSITION_SAFETY = 325;
    public static final double TICKS_IN_DEG = (double) 360/288*72/45; // =
    public static final double TICKS_IN_DEG_ELEVATOR = 19.0675;
    public static final int MAX_HORZ_POS = -750;
    public static int ELEVATOR_BOTTOM_POSITION = 30, ELEVATOR_BOTTOM_POSITION_LEFT = 30, ELEVATOR_MIDDLE_POSITION = 1700, ELEVATOR_TOP_POSITION = 2550;
    public static final double LOGRIPPER_CLOSE_POS = 0.4, LOGRIPPER_A_BIT_OPEN_POS= 0.6, LOGRIPPER_OPEN_POS = 0.75;
    public static final double HIGRIPPER_CLOSE_POS = 0.89, HIGRIPPER_OPEN_POS = 0.35;
    public static final double sweepStart = 0.95, sweepEnd = 0.5;
    public static final double InRotPosUp = 0, InRotPosMid =0.55, InRotPosDown = 1;
    public static double elevatorP = 0.01, elevatorI = 0.0, elevatorD = 0.00, elevatorF = 0.35;

    // Gripper Spinner
    public static final double GRIPPER_SPINNER_UP = 0.9;
    public static final double GRIPPER_SPINNER_MID = 0.55;
    public static final double GRIPPER_SPINNER_DOWN = 0.125;

    // Low Gripper
    public static final double LOW_GRIPPER_OPENED = 0.2;
    public static final double LOW_GRIPPER_CLOSED = 0.34;
}