package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    public int targetArm, currentPos;

    public double power;
    private PIDController controller;
    public DcMotor arm;

    public highGripper HighGripper;

//    public MultipleTelemetry telemetry;

//    public static int target;
    public Arm(HardwareMap hardwareMap){
        arm = hardwareMap.get(DcMotor.class, "arm");
        controller = new PIDController(Constants.armP, Constants.armI, Constants.armD);
        targetArm = Constants.ARM_INIT;
        HighGripper = new highGripper(hardwareMap);

//        int currentPos = arm.getCurrentPosition();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public void handleArm(Gamepad gamepad) {

        currentPos = arm.getCurrentPosition();

//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        controller.setPID(Constants.armP, Constants.armI, Constants.armD);



        if (gamepad.circle) {
            while(targetArm != Constants.ARM_IN){
                if(targetArm>Constants.ARM_IN)targetArm--;
                else targetArm++;
            }
//            arm.setTargetPosition(Constants.ARM_UP);
//            target = Constants.ARM_UP;
        }
        if (gamepad.square) {
            while(targetArm != Constants.ARM_OUT){
                if(targetArm>Constants.ARM_OUT)targetArm--;
                else targetArm++;
            }
//            HighGripper.high_gripper.setPosition(Constants.HIGRIPPER_OPEN_POS);
//            HighGripper.padLock = true;
//            HighGripper.isOpen = true;
//            arm.setTargetPosition(Constants.ARM_SIDE);
//            target = Constants.ARM_SIDE;
        }
        if (gamepad.left_stick_button){
            while(targetArm != Constants.ARM_MOVING){
                if(targetArm>Constants.ARM_MOVING)targetArm--;
                else targetArm++;
            }
        }

        double pid = controller.calculate(currentPos, targetArm);
        double ff = Math.cos(Math.toRadians(targetArm/Constants.TICKS_IN_DEG)) * Constants.armF;
        power = 0.75*(pid + ff);
        arm.setPower(power);


//        telemetry.addData("arm pos", currentPos);
//        telemetry.addData("arm target", targetArm);
//        telemetry.update();

    }

}
