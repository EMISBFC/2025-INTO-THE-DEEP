package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;

public class LConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches = 0.002938849934989059;
        ThreeWheelIMUConstants.strafeTicksToInches = 0.002956112709665143;
        ThreeWheelIMUConstants.turnTicksToInches = 0.0019815495361425726;
        ThreeWheelIMUConstants.leftY = 6.7;
        ThreeWheelIMUConstants.rightY = -6.7;
        ThreeWheelIMUConstants.strafeX = -6.3;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = ConstantNamesHardwaremap.WHEELBACKLEFT;
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = ConstantNamesHardwaremap.WHEELFRONTRIGHT;
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = ConstantNamesHardwaremap.WHEELBACKRIGHT;
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
    }
}




