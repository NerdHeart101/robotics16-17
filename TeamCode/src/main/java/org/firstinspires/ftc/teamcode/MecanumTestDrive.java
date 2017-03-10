package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by jonathon on 1/19/17.
 */

@TeleOp(name="Analog Stick Test", group="Compbot")

public class MecanumTestDrive extends OpMode {

    // Access the robot
    // HardwareCompbot robot = new HardwareCompbot();

    // Powers (speeds) for each motor
    final double DRIVE_POWER    = 0.6;
    final double INTAKE_POWER   = 0.2;
    final double ELEVATOR_POWER = 1.0;
    final double KICKER_POWER   = 1.0;

    @Override
    public void init() {

        // Initialize the robot
        // robot.init(hardwareMap);

        // Notify the drivers
        telemetry.addData("Say","Hello Drivers");
    }

    public void loop() {

        telemetry.addData("Left Stick","%d x : %d y",gamepad1.left_stick_x,gamepad1.left_stick_y);
        telemetry.addData("Right Stick","%d x : %d y",gamepad1.right_stick_x,gamepad1.right_stick_y);
        telemetry.update();

        /*
        // Current speeds of the motors
        double frontLeft,backLeft,frontRight,backRight;
        double intake,elevator,kicker;

        // Driver 1 - Driving and intake

        // Run wheels in mecano mode
        // Left stick for rotation, right for movement
        // TODO: add commands (maybe a method?) to control the wheels

        // Intake - A or right trigger for intake, B or right bumper for
        intake = !gamepad1.right_bumper ? gamepad1.right_trigger * INTAKE_POWER : -INTAKE_POWER;

        // Driver 2 - Elevator and kicker

        // Elevator - Left trigger for forward, left bumper for reverse
        elevator = (gamepad2.left_trigger != 0) ? (gamepad2.left_trigger * ELEVATOR_POWER)
                : gamepad2.left_bumper ? -ELEVATOR_POWER : 0.0;

        // Kicker - Right trigger for forward, right bumper for reverse
        kicker = gamepad2.right_trigger != 0 ? gamepad2.right_trigger * KICKER_POWER
                : gamepad2.right_bumper ? -KICKER_POWER : 0.0;

        // Set power of all motors to the correct value

        robot.intakeMotor.setPower(intake);
        robot.elevatorMotor.setPower(elevator);
        robot.kickerMotor.setPower(kicker);

        // Send telemetry message to signify robot running
        // TODO: add degree indicator for bot movement and rotation
        telemetry.addData("intake",   "%.2f", intake);
        telemetry.addData("elevator", "%.2f", elevator);
        telemetry.addData("kicker",   "%.2f", kicker);
        */
    }
}
