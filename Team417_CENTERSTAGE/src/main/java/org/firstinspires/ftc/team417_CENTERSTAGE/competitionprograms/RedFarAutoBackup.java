package org.firstinspires.ftc.team417_CENTERSTAGE.competitionprograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.League1BaseAutonomous;

@Autonomous (name="Red Far Auto Backup ")

public class RedFarAutoBackup extends League1BaseAutonomous {
    public void runOpMode() {
        runSimpleInchesAuto(true, false);
    }
}
