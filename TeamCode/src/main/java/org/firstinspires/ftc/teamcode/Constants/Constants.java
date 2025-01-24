package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static double armP = 0.02, armI = 0.025, armD= 0.001, armF = 0.275;
    public static final double ARMTOLERANCE = 5;
    public static final int ARM_IN = 315, ARM_OUT = 110, ARM_INIT = 0, ARM_MOVING = 125, ARM_FUCKED = 305;
    public static final double TICKS_IN_DEG = (double) 360/288*72/45; // =
    public static final double TICKS_IN_DEG_ELEVATOR = 19.0675;

    public static final int MAX_HORZ_POS = 930; // SET  TO 850 AFTER ROBOT LEGAL K THANKS BYE, 150 is legal

    public static final double LOGRIPPER_CLOSE_POS = 0.405, LOGRIPPER_OPEN_POS = 0.75;
    public static final double HIGRIPPER_CLOSE_POS = 0.89, HIGRIPPER_OPEN_POS = 0.35;

    public static final double InRotPosUp = 0.2, InRotPosMid =0.55, InRotPosDown = 0.9;

    public static double elevatorP = 0.005, elevatorI = 0.0, elevatorD = 0.00, elevatorF = 0.5;
}