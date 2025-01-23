package org.firstinspires.ftc.teamcode.Autos;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;
public class AutoArm {

    public static final int TRANSITION_POSITION = (int) (40 * Constants.TICKS_IN_DEG);
    public static final int GRAB_POSITION = (int) (160 * Constants.TICKS_IN_DEG);

    private final DcMotor armMotor;
    private final PIDController pidController;
    private int targetPosition;

    public AutoArm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, ConstantNamesHardwaremap.ARM);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidController = new PIDController(Constants.armP, Constants.armI, Constants.armD);
        targetPosition = 0;
    }}