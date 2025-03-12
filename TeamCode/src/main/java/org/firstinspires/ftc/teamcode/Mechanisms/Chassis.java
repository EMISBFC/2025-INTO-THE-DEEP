package org.firstinspires.ftc.teamcode.Mechanisms;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;

public class Chassis {
    public Motor fl;
    public Motor fr;
    public Motor bl;
    public Motor br;
    public static RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    public IMU imu;
    MecanumDrive mecanum;
    public Chassis(HardwareMap hardwareMap){

        fl = new Motor(hardwareMap, ConstantNamesHardwaremap.WHEELFRONTLEFT);
        fr = new Motor(hardwareMap, ConstantNamesHardwaremap.WHEELFRONTRIGHT);
        bl = new Motor(hardwareMap, ConstantNamesHardwaremap.WHEELBACKLEFT);
        br = new Motor(hardwareMap, ConstantNamesHardwaremap.WHEELBACKRIGHT);
//        br.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mecanum = new MecanumDrive(fl, fr, bl, br);
        imu = hardwareMap.get(IMU.class, "imu");
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }


    public void fieldCentricDrive(double x, double y, double rx, boolean bumper){
        if(bumper) {
            mecanum.driveFieldCentric(x * 0.3, y * 0.3, rx * 0.3, getHeading());
        }
        else{
            mecanum.driveFieldCentric(x * 1, y * 1, rx * 1, getHeading());
        }
    }
    public void robotCentricDrive(double x, double y, double rx, boolean bumper){
        if(bumper){
            mecanum.driveRobotCentric(x*0.8, y*0.8, rx*0.8);
        }
        else{
            mecanum.driveRobotCentric(x*1, y*1, rx*1);
        }
    }
}
