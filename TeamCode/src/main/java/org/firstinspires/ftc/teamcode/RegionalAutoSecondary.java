package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Maint on 2/25/2017.
 */

@Autonomous(name="RED: Match #21",group="Regionals")
public class RegionalAutoSecondary extends AutonomousBase {

    @Override
    public void runOpMode() {

        initializeAutonomous();

        waitTime(10);

        encoderDrive(DRIVE_SPEED,-24,-24,4);

        launchBall();
        nextBall();
        launchBall();

        encoderDrive(TURN_SPEED,-5,5,2);
        encoderDrive(DRIVE_SPEED,-50,-50,6);

    }
}
