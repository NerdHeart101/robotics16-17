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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This code is to drive the competition bot using two drivers controlling different aspects of the bot
 */

@TeleOp(name="Teleop Tank (Competition)", group="Compbot")

public class TeleOpMecanumComp extends OpMode{

    // Access the robot
    HardwareCompbot robot   = new HardwareCompbot();

    // Powers (speeds) for each motor
    double DRIVE_POWER      = 1.0;
    double INTAKE_POWER     = 0.2;
    double ELEVATOR_POWER   = 1.0;
    double KICKER_POWER     = 1.0;
    double CAPPER_POWER     = 1.0;

    @Override
    public void init() {

        // Initialize the robot
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Drivers");    //
    }

    @Override
    public void loop() {
        double speed,angle,rotate,intake,elevator,kicker,capper,buttonS,capperS;

        // Driver 1 - Driving, intake, button pusher

        // Run wheels in arcade mode
        speed = Math.hypot(gamepad1.left_stick_x,gamepad1.left_stick_y);
        angle = Math.atan2(gamepad1.left_stick_x,gamepad1.left_stick_y) + 180;
        rotate = gamepad1.right_stick_x;

        // Intake - RT for in, RB for out
        intake = !gamepad1.right_bumper ? gamepad1.right_trigger * INTAKE_POWER : -INTAKE_POWER;

        // Button Pusher - LT for out, LB for in
        buttonS = gamepad1.left_trigger > 0 ? 1.0 : gamepad1.left_bumper ? 0.0 : robot.buttonPusher.getPosition();

        // Driver 2 - Elevator, kicker, capper

        // Elevator - Left trigger for forward, left bumper for reverse
        elevator = (gamepad2.left_trigger != 0) ? (gamepad2.left_trigger * ELEVATOR_POWER)
                : gamepad2.left_bumper ? -ELEVATOR_POWER : 0.0;

        // Kicker - Right trigger for forward, right bumper for reverse
        kicker = gamepad2.right_trigger != 0 ? gamepad2.right_trigger * KICKER_POWER
                : gamepad2.right_bumper ? -KICKER_POWER : 0.0;

        // Capper - D-pad up and down for control, D-pad left or right for servo
        capper = gamepad2.dpad_up ? CAPPER_POWER : gamepad2.dpad_down ? -CAPPER_POWER : 0.0;

        capperS = gamepad2.dpad_left ? 0.5 : gamepad2.dpad_right ? 0.0 : robot.capperPusher.getPosition();

        // Set power of all motors to the correct value
        robot.frontRight.setPower(DRIVE_POWER * speed * Math.sin(angle) + rotate);
        robot.backRight.setPower(DRIVE_POWER * speed * Math.cos(angle) + rotate);
        robot.frontLeft.setPower(DRIVE_POWER * speed * Math.cos(angle) - rotate);
        robot.backLeft.setPower(DRIVE_POWER * speed * Math.sin(angle) - rotate);
        robot.intakeMotor.setPower(intake);
        robot.elevatorMotor.setPower(elevator);
        robot.kickerMotor.setPower(kicker);
        robot.capperMotor.setPower(capper);

        // Set position of both servos to the correct value
        robot.buttonPusher.setPosition(buttonS);
        robot.capperPusher.setPosition(capperS);

        // Send telemetry message to signify robot running;
        telemetry.addData("speed",    "%.2f", speed);
        telemetry.addData("angle",    "%.2f", angle);
        telemetry.addData("intake",   "%.2f", intake);
        telemetry.addData("elevator", "%.2f", elevator);
        telemetry.addData("kicker",   "%.2f", kicker);
    }
}
