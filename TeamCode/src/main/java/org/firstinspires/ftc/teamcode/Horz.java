package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Horz{
    //private Motor horz;
    public DcMotor horz;


    private final double MAX_SPEED = 1.0;
    private final int MAX_EXTENSION_POSITION = 840;

    public void loop(){
        telemetry.addData("Encoder Position", horz.getCurrentPosition());
        telemetry.update();
    }
    public Horz(HardwareMap hardwareMap){

        //horz = new Motor(hardwareMap, "horz");
        horz = hardwareMap.get(DcMotor.class, "horz");
        horz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void handleHorz(Gamepad gamepad) {
        /*double rightTrigger = gamepad.right_trigger;  // Right trigger for forward
        double leftTrigger = gamepad.left_trigger;

        if(leftTrigger>0.3){
            horz.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            horz.motor.setPower(0.65);
        }

        if(gamepad.right_trigger>0.3){
            horz.motor.setDirection(DcMotorSimple.Direction.FORWARD);
            horz.motor.setPower(0.65);
        }
        else if (gamepad.left_trigger<0.3 && gamepad.right_trigger<0.3){
            horz.motor.setPower(0);
        }
    }*/
        double leftTrigger = gamepad.left_trigger;
        double rightTrigger = gamepad.right_trigger;

        double power = rightTrigger - leftTrigger;
        double motorPower = Range.clip(power, -MAX_SPEED, MAX_SPEED);

        int currentPos = horz.getCurrentPosition();

        if (motorPower > 0 && currentPos < MAX_EXTENSION_POSITION){
            horz.setPower(motorPower);
        } else if (motorPower < 0) {
            horz.setPower(motorPower);
        } else {
            horz.setPower(0);
        }


    }
}