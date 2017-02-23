package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Maint on 2/23/2017.
 */

@Autonomous(name="Garrett's Auto", group="Compbot")

public class GarrettAutoRed extends AutonomousBase {

    @Override
    public void runOpMode() {

        initializeAutonomous();

        encoderDrive(DRIVE_SPEED, -8, -8, 2);

        launchBall();
        nextBall();
        launchBall();

        encoderDrive(TURN_SPEED, -4.8, 4.8, 2.0);
        encoderDrive(TURN_SPEED, -64, -64, 5.0);



    }
}
