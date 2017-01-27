package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by jonathon on 1/27/17.
 */

@Autonomous(name="BLUE: Jan 28 meet",group="comp")

public class Jan28Auto_Blue extends AutonomousBase {

    @Override
    public void runOpMode() {

        // TODO: make the nextBall method more precise
        initializeAutonomous();

        launchBall();
        nextBall();
        launchBall();

        encoderDrive(DRIVE_SPEED,20,20,1.7);
        encoderDrive(TURN_SPEED,-20,20,2);
        encoderDrive(DRIVE_SPEED,-6,-6,0.4);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
