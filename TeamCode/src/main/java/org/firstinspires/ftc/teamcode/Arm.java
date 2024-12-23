package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Constants;
public class Arm {
    public int target, currentPos;
    private PIDController controller;
    public DcMotor arm;
    public Arm(HardwareMap hardwareMap){
        arm = hardwareMap.get(DcMotor.class, "arm");
        controller = new PIDController(Constants.armP, Constants.armI, Constants.armD);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setPower(0.4);
    }
    public void handleArm(Gamepad gamepad) {

        if (gamepad.circle) {
            arm.setTargetPosition(Constants.ARM_UP);
            target = Constants.ARM_UP;
        }
        if (gamepad.square) {
            arm.setTargetPosition(Constants.ARM_SIDE);
            target = Constants.ARM_SIDE;
        }
        currentPos = arm.getCurrentPosition();
        double pid = controller.calculate(currentPos, target);
//        double ff = Math.cos(Math.toRadians(target/Constants.TICKS_IN_DEG)) * Constants.armF;
//        double power = 0.75*(pid + ff);
        arm.setPower(pid);
    }

}
