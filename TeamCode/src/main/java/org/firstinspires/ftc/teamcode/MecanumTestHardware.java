package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumTestHardware {

    public DcMotor frontRight   = null;
    public DcMotor backRight    = null;
    public DcMotor frontLeft    = null;
    public DcMotor backLeft     = null;

    HardwareMap hwMap           = null;
    private ElapsedTime period  = new ElapsedTime();

    public MecanumTestHardware() {

    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        frontRight  = hwMap.dcMotor.get("front_right_drive");
        backRight   = hwMap.dcMotor.get("back_right_drive");
        frontLeft   = hwMap.dcMotor.get("front_left_drive");
        backLeft    = hwMap.dcMotor.get("back_left_drive");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long)period.milliseconds();

        if(remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        period.reset();
    }
}
