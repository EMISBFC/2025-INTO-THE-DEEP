package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.ConstantNamesHardwaremap;
import org.firstinspires.ftc.teamcode.Constants.Constants;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL_IMU;

        FollowerConstants.leftFrontMotorName = ConstantNamesHardwaremap.WHEELFRONTLEFT;
        FollowerConstants.leftRearMotorName = ConstantNamesHardwaremap.WHEELBACKLEFT;
        FollowerConstants.rightFrontMotorName = ConstantNamesHardwaremap.WHEELFRONTRIGHT;
        FollowerConstants.rightRearMotorName = ConstantNamesHardwaremap.WHEELBACKRIGHT;

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 12;

        FollowerConstants.xMovement = 61.817352299693404;
        FollowerConstants.yMovement = 44.67930478502683;

        FollowerConstants.forwardZeroPowerAcceleration = -34.36973498827856;
        FollowerConstants.lateralZeroPowerAcceleration = -71.36158107646105;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(5,0.01,0.075,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.25,0.0005,0.029,0);

        FollowerConstants.headingPIDFCoefficients.setCoefficients(0.5,0,0.09,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(1.8,0,0.1,0);

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.015,0,0.0005,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.008,0,0.0008,0.6,0);

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
