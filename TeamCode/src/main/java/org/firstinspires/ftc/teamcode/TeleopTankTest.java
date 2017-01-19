/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This code is to drive the competition bot using either one or two drivers
 * Each of the drivers controls all aspects of the bot
 * If two inputs for the same motor are received, the player normally with that control takes priority
 */

@TeleOp(name="Teleop Tank (Testing)", group="Pushbot")

public class TeleopTankTest extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.

    /*
     * Code to run ONCE when the driver hits INIT
     */

    double DRIVE_POWER = .6;
    double INTAKE_POWER = .2;
    double ELEVATOR_POWER = 1;
    double KICKER_POWER = 1;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left,leftPrimary,leftSecondary;
        double right,rightPrimary,rightSecondary;
        double intake,intakePrimary,intakeSecondary;
        double elevator,elevatorPrimary,elevatorSecondary;
        double kicker,kickerPrimary,kickerSecondary;

        // Driver 1 - Driving and intake

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        leftPrimary = gamepad1.left_stick_y * DRIVE_POWER;
        rightPrimary = gamepad1.right_stick_y * DRIVE_POWER;

        // Intake - A or right trigger for intake, B or right bumper for output
        // If allowAlternateControls is true, then RT and RB are not available, so adjust.
        // Holy nested if-else Batman!
        intakePrimary = gamepad1.a ? INTAKE_POWER : gamepad1.b ? -INTAKE_POWER : 0.0;

        // Driver 2 - Elevator and kicker

        // Elevator - Left trigger for forward, left bumper for reverse
        elevatorPrimary = (gamepad2.left_trigger != 0) ? (gamepad2.left_trigger * ELEVATOR_POWER)
                : gamepad2.left_bumper ? -ELEVATOR_POWER : 0.0;

        // Kicker - Right trigger for forward, right bumper for reverse
        kickerPrimary = gamepad2.right_trigger != 0 ? gamepad2.right_trigger * KICKER_POWER
                : gamepad2.right_bumper ? -KICKER_POWER : 0.0;


        // Alternate controls allow for each driver to perform actions delegated to the other
        // Driver 1 Alternate - Elevator and kicker

        // Elevator - Left trigger for forward, left bumper for reverse
        elevatorSecondary = gamepad1.left_trigger != 0 ? gamepad1.left_trigger * ELEVATOR_POWER
                    : gamepad1.left_bumper ? -ELEVATOR_POWER : 0.0;

        // Kicker - Right trigger for forward, right bumper for reverse
        kickerSecondary = gamepad1.right_trigger != 0 ? gamepad1.right_trigger * KICKER_POWER
                    : gamepad1.right_bumper ? -KICKER_POWER : 0.0;

        // Driver 2 Alternate - Driving and intake

        // Run wheels in tank mode
        leftSecondary = gamepad2.left_stick_y;
        rightSecondary = gamepad2.right_stick_y;

        // Intake - A for intake, B button for output (Note that trigger controls are not applied)
        intakeSecondary = gamepad2.a ? INTAKE_POWER : gamepad2.b ? -INTAKE_POWER : 0.0;

        // Resolve conflicts of primary and alternate controls
        left = leftPrimary != 0 ? leftPrimary : leftSecondary;
        right = rightPrimary != 0 ? rightPrimary : rightSecondary;
        intake = intakePrimary != 0 ? intakePrimary : intakeSecondary;
        elevator = elevatorPrimary != 0 ? elevatorPrimary : elevatorSecondary;
        kicker = kickerPrimary != 0 ? kickerPrimary : kickerSecondary;

        // Set power of all motors to the correct value
        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);
        robot.intakeMotor.setPower(intake);
        robot.elevatorMotor.setPower(elevator);
        robot.kickerMotor.setPower(kicker);

        // Send telemetry message to signify robot running;
        telemetry.addData("left",     "%.2f", left);
        telemetry.addData("right",    "%.2f", right);
        telemetry.addData("intake",   "%.2f", intake);
        telemetry.addData("elevator", "%.2f", elevator);
        telemetry.addData("kicker",   "%.2f", kicker);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
