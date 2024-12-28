package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
//    public static double armP = 2.5, armI = 0.75, armD= 0.5, armF = 0.225;

    public static double armP = 0.02, armI = 0.025, armD= 0.001, armF = 0.275;
    public static final int ARM_UP = 150, ARM_SIDE = 250, ARM_INIT = 0;
    public static final double TICKS_IN_DEG = (double) 360/288*72/45; // = 2

    public static final double LOGRIPPER_CLOSE_POS = 0.51, LOGRIPPER_OPEN_POS = 0.9;
    public static final double HIGRIPPER_CLOSE_POS = 0.4, HIGRIPPER_OPEN_POS = 0.95;

    public static final double InRotPosUp = 0.1, InRotPosMid =0.5, InRotPosDown = 0.9;
//    public static int initialArmPos = 0, armPosFloor = 2350, armPosHang = 150, armPosBackdrop = -800;
}