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
 * This code is to drive the competition bot using two drivers controlling different aspects of the bot
 */

@TeleOp(name="Teleop Tank (Competition)", group="Pushbot")

public class TeleopTankComp extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.

    /*
     * Code to run ONCE when the driver hits INIT
     */

    double DRIVE_POWER = .6;
    double INTAKE_POWER = .15;
    double ELEVATOR_POWER = 1;
    double KICKER_POWER = 1;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Drivers");    //
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
        double left;
        double right;
        double intake;
        double elevator;
        double kicker;

        // Driver 1 - Driving and intake

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y * DRIVE_POWER;
        right = gamepad1.right_stick_y * DRIVE_POWER;

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
