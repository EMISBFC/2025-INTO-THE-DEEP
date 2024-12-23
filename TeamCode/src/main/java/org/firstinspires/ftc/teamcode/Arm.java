package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;

public class Arm {
    public DcMotor arm;

    public void loop(){
        telemetry.addData("Arm Position", arm.getCurrentPosition());
        telemetry.update();
    }
    public Arm(HardwareMap hardwareMap){
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void handleArm(Gamepad gamepad) {

        if (gamepad.circle ) {
            arm.setTargetPosition(Constants.ARM_UP);
        }

        if (gamepad.square){
            arm.setTargetPosition(Constants.ARM_SIDE);
        }

    }

}
