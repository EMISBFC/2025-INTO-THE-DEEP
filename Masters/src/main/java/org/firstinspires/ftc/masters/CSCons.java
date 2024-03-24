package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
public class CSCons {

    public static double skewampus = 0;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double pixelDetectThreshold = 350;
    public static double clawOpen = 0.2;
    public static double clawClosed = 0.57;
    public static double clawTransfer = 0.38; // .65
    public static double clawAngleGroundToThree = 1; //.9?
    public static double clawAngleFourToFive = .98;
    public static double clawAngleTransition = .34;
    public static double clawAngleTransfer = .12;
    public static double clawArmGround = 0.82;//.78;
    public static double clawArm2 = .75;
    public static double clawArm3 = .72;
    public static double clawArm4 = .65;
    public static double clawArm5 = .62;
    public static double clawArmTransition = .23;
    public static double clawArmTransfer = .24;

    //public static double[]outtakeAngle={0.234, 0.9}; //folder, transfer
    public static double outtakeAngleFolder=0;
    public static double outtakeAngleTransfer=.62;
    //test public static double outtakeAngleTransfer=.53;
    //public static double[] doubleServoBack= {0.7, 0.15}; //drop, transfer
    public static double outtakeMovementBackDrop =0.25;
    public static double outtakeMovementTransfer = .72;
    //public static double[] rightSideBack = {0.955, 0.65};
    public static double openHook = 0.6;
    public static double closeHook = 1;
    public static double openMicroHook =0.5;
    public static double closeMicroHook =0;

    public static double droneFlat = 0.59;
    public static double droneShooting = 0.2;

    public static double closeClawDistance = 3; //in cm

    public static long transferToBottomIntake = 200; //time in ms
    public static long transferToScoreOuttake = 200;
    public static long scoreToTransferOuttake = 200;
    public static long closingHook = 100;

    public static int leftIntakeExtension = 990;
    public static int centerIntakeExtension = 1240;
    public static int rightIntakeExtension = 620;

    public static int redRightIntakeExtension= 660;
    public static int redCenterIntakeExtension = 1240;
    public static int getLeftIntakeExtension = 960;


    public static double backMultiplier =0.78;
    public static double frontMultiplier = 1;

    public static double servo1Up = 0.4;
    public static double servo1Down = 0.65;
    public static double servo2Up =0.6;
    public static double servo2Down= 0.35;

    public static double wristCenter = 0.5;


    public enum OuttakePosition{
        BOTTOM (-10),
        LOW_AUTO(105),
        AUTO(900),
        LOW(1400),
        MID(2400),
        HIGH(3400);

        private int target;

        private OuttakePosition(int target){
            this.target = target;
        }

        public int getTarget(){
            return  target;
        }
    }

    public enum IntakeState {
        Transition, Transfer, Intake, MoveToIntake, MoveToTransfer;
    }

    public enum OuttakeState{
        ClosingHook, MoveToTransfer, ReadyToTransfer, MoveToDrop, ReadyToDrop, Align, BackUp
    }

    public enum DriveMode {
        NORMAL,
        PIXEL_SCORE,
        END_GAME
    }

    public enum ClawPosition{
        OPEN, CLOSED, TRANSFER
    }

    public enum  HookPosition{
        OPEN, CLOSED, HALF
    }

/**
 *
 *  ╚ ╔ ╩ ╦ ╠ ═ ╬ ╣ ║ ╗ ╝
 *      THE WISDOM!
 *  T H E   W I S D O M !
 *                   ║
 *           ╔═══════╣
 *           ║  ╔════╣
 *           ║  ║    ║
 *           ║  ║    ║
 *                   ║
 *  **/
}
