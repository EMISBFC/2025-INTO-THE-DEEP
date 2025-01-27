package org.firstinspires.ftc.teamcode.Mechanisms;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;

public class Chassis {
    private Motor fl;
    private Motor fr;
    private Motor bl;
    private Motor br;
    public static RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    IMU imu;
    MecanumDrive mecanum;
    public Chassis(HardwareMap hardwareMap){

        fl = new Motor(hardwareMap, ConstantNamesHardwaremap.WHEELFRONTLEFT);
        fr = new Motor(hardwareMap, ConstantNamesHardwaremap.WHEELFRONTRIGHT);
        bl = new Motor(hardwareMap, ConstantNamesHardwaremap.WHEELBACKLEFT);
        br = new Motor(hardwareMap, ConstantNamesHardwaremap.WHEELBACKRIGHT);
        fr.motor.setDirection(DcMotorSimple.Direction.REVERSE);
//        br.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mecanum = new MecanumDrive(fl, fr, bl, br);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }


    public void fieldCentricDrive(double x, double y, double rx, double acc){
        if(acc>0.5) {
            mecanum.driveFieldCentric(x * 1, y * 1, rx * 1, getHeading());
        }
        else{
            mecanum.driveFieldCentric(x * 0.75, y * 0.75, rx * 0.75, getHeading());
        }
    }
    public void robotCentricDrive(double x, double y, double rx, double acc){
        if(acc>0.1){
            mecanum.driveRobotCentric(x*1, y*1, rx*1);
        }
        else{
            mecanum.driveRobotCentric(x*0.8, y*0.8, rx*0.8);
        }
    }
}
