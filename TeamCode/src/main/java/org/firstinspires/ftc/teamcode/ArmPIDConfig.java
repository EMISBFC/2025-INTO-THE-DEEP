package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmPIDConfig {

    //    public static double armP = 0.005, armI = 0, armD= 0.001, armF = 0.225;
    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0.001;

    public static double TRANSITION_POSITION_DEGREES = -150.0;
    public static double SPECIMEN_POSITION_DEGREES = -50.0;
    public static double BASKET_POSITION_DEGREES = 50.0;
    public static double PLAYER_POSITION_DEGREES = 150.0;
}

