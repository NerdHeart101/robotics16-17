package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareCompbot {

    /* Public OpMode members. */
    public DcMotor  frontRight      = null;
    public DcMotor  backRight       = null;
    public DcMotor  frontLeft       = null;
    public DcMotor  backLeft        = null;
    public DcMotor  elevatorMotor   = null;
    public DcMotor  kickerMotor     = null;
    public DcMotor  intakeMotor     = null;

    public Servo    buttonPusher    = null;
    public Servo    intakePusher    = null;

    public ModernRoboticsI2cRangeSensor rangeSensor = null;
    public ColorSensor                  colorSensor = null;
    public OpticalDistanceSensor        odsSensor   = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareCompbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontRight      = hwMap.dcMotor.get("front_right_drive");
        backRight       = hwMap.dcMotor.get("back_right_drive");
        frontLeft       = hwMap.dcMotor.get("front_left_drive");
        backLeft        = hwMap.dcMotor.get("back_left_drive");
        elevatorMotor   = hwMap.dcMotor.get("elevator");
        kickerMotor     = hwMap.dcMotor.get("kicker");
        intakeMotor     = hwMap.dcMotor.get("intake");

        // Define and Initialize Servos
        //buttonPusher    = hwMap.servo.get("button_pusher");
        //intakePusher    = hwMap.servo.get("intake_pusher");
        
        // Define and Initialize Sensors
        //colorSensor     = hwMap.colorSensor.get("sensor_color");
        //odsSensor       = hwMap.opticalDistanceSensor.get("sensor_ods");
        //rangeSensor     = hwMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        elevatorMotor.setPower(0);
        kickerMotor.setPower(0);
        intakeMotor.setPower(0);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

