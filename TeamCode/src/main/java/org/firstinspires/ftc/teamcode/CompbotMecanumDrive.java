package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by jonathon on 1/19/17.
 */

@TeleOp(name="Mecanum Drive (Competition)", group="Pushbot")
@Disabled
public class CompbotMecanumDrive extends OpMode {

    HardwarePushbot robot = new HardwarePushbot();

    final double DRIVE_POWER    = 0.6;
    final double INTAKE_POWER   = 0.2;
    final double ELEVATOR_POWER = 1.0;
    final double KICKER_POWER   = 1.0;

    @Override
    public void init() {

        robot.init(hardwareMap);
        telemetry.addData("Say","Hello Driver");
    }

    public void loop() {
        //Driving code goes here
    }
}
