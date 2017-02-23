package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Maint on 2/23/2017.
 */

@Autonomous(name="Garrett's Auto", group="Compbot")

public class GarrettAutoRed extends AutonomousBase {

    public static boolean isPushing = true;;

    @Override
    public void runOpMode() {

        initializeAutonomous();

        encoderDrive(DRIVE_SPEED, -8, -8, 2);
       /* launchBall();
        nextBall();
        launchBall();
        */

        encoderDrive(TURN_SPEED, -4.3, 4.3, 2.0);

        while(robot.odsSensor.getRawLightDetected() < 1 && opModeIsActive()) {
            robot.leftMotor.setPower(-0.4);
            robot.rightMotor.setPower(-0.4);
            telemetry.addData("Robot","Moving to white line");
            telemetry.addData("Raw",    robot.odsSensor.getRawLightDetected());
            telemetry.addData("Normal", robot.odsSensor.getLightDetected());
            telemetry.update();
        }

        encoderDrive(TURN_SPEED, -5.8, 5.8, 2.0);

        pushButton(true);






    }
}
