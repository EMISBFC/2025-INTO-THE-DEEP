package org.firstinspires.ftc.teamcode.Mechanisms;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;

public class Horz{
    //private Motor horz;
    public DcMotor horz;
    TouchSensor limit_switch;
    public boolean clicked;
    private final double MAX_SPEED = 1.0;
//    private final int MAX_EXTENSION_POSITION = 840;
    public int zero_position = 0;
    public Horz(HardwareMap hardwareMap){

        //horz = new Motor(hardwareMap, "horz");
        horz = hardwareMap.get(DcMotor.class, ConstantNamesHardwaremap.HORZ);
        horz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        horz.setDirection(DcMotorSimple.Direction.REVERSE);
        limit_switch = hardwareMap.get(TouchSensor.class, ConstantNamesHardwaremap.LIMITSWITCH  );
    }

    public void handleHorz(Gamepad gamepad) {
        if (!limit_switch.isPressed()) {
            zero_position = horz.getCurrentPosition(); // Set the zero position when switch is pressed
        }

        double leftTrigger = gamepad.left_trigger;
        double rightTrigger = gamepad.right_trigger;

        double power = rightTrigger - leftTrigger;
        double motorPower = Range.clip(power, -MAX_SPEED, MAX_SPEED);

        int currentPos = horz.getCurrentPosition();

        clicked = limit_switch.isPressed();


        if (motorPower > 0 && limit_switch.isPressed()) { //  && currentPos >= zero_position
            horz.setPower(motorPower );
        }else if (motorPower > 0 && currentPos < zero_position+Constants.MAX_HORZ_POS + 400) {
            horz.setPower(0.1);
        }
        else if (motorPower < 0 && currentPos > zero_position+Constants.MAX_HORZ_POS) {
            horz.setPower(motorPower);
        } else {
            horz.setPower(0);
        }

    }
}