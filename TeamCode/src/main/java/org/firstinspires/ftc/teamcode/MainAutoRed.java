package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Maint on 2/7/2017.
 */
@Autonomous(name="Autonomous Main with Sensors", group="Compbot")
public class MainAutoRed extends AutonomousBase {

    HardwareCompbot robot = new HardwareCompbot();



    @Override
    public void runOpMode() {

        initializeAutonomous();
        launchBall();
        nextBall();
        launchBall();





    }
}
