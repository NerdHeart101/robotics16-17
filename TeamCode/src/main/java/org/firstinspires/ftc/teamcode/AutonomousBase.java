package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// TODO : Redo all of the auto everything
public class AutonomousBase extends LinearOpMode {

    // Robot Items
    HardwareCompbot robot = new HardwareCompbot();
    protected ElapsedTime runtime = new ElapsedTime();

    // Motor Items
    static final double COUNTS_PER_MOTOR_REV    = 1440;
    static final double DRIVE_GEAR_REDUCTION    = 1.0;
    static final double DRIVE_SPEED             = 1.0;
    static final double TURN_SPEED              = 0.6;

    // Robot measurements
    static final double WHEEL_DIAMETER_INCHES   = 4.0;
    static final double ROBOT_RADIUS            = 8.15;

    // Encoder information
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                                                / (WHEEL_DIAMETER_INCHES * Math.PI * Math.sqrt(2.0));

    @Override
    public void runOpMode() {
        telemetry.addData("Error", "This should not be seen. Check the AutonomousBase class");
        telemetry.update();
    }

    public void initializeAutonomous() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Please Wait: Resetting Encoders");    //
        telemetry.update();

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Ready to Begin: Encoders Reset");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
    }

    public void driveToLine(double angle, double timeout) {
        angle+=45;
        if(opModeIsActive()) {

            robot.frontLeft.setPower(DRIVE_SPEED / 2 * Math.cos(Math.toRadians(angle)));
            robot.frontRight.setPower(DRIVE_SPEED / 2 * Math.sin(Math.toRadians(angle)));
            robot.backLeft.setPower(DRIVE_SPEED / 2 * Math.sin(Math.toRadians(angle)));
            robot.backRight.setPower(DRIVE_SPEED / 2 * Math.cos(Math.toRadians(angle)));

            while (opModeIsActive() && runtime.seconds() < timeout) {
                telemetry.addData("Status", "Driving to Line");
                telemetry.addData("ODS Data", "%.4f",robot.odsSensor.getLightDetected());
                telemetry.addData("ODS Raw", "%.4f",robot.odsSensor.getRawLightDetected());
                telemetry.addData("Timeout", "%.2f : %.2f", timeout, runtime.seconds());
                telemetry.update();
                if (robot.odsSensor.getRawLightDetected() > 1.0) {
                    break;
                }
            }

            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }
    }

    public void pushButton(boolean pushing) {
        if (pushing) {
            robot.buttonPusher.setPosition(1.0);
        } else {
            robot.buttonPusher.setPosition(0.0);
        }
        runtime.reset();
        while (runtime.milliseconds() < 500 && opModeIsActive()) {
            telemetry.addData("Status", "Pushing Button : %b", pushing);
            telemetry.update();
        }
    }
    public void encoderDrive(double inches, double angle) {
        encoderDrive(inches, angle, DRIVE_SPEED, inches / DRIVE_SPEED);
    }
    public void encoderDrive(double inches, double angle, double speed) {
        encoderDrive(inches, angle, speed, inches / DRIVE_SPEED);
    }
    public void encoderDrive(double inches, double angle, double speed, double timeout) {
        int fl, fr, bl, br;
        if(opModeIsActive()) {

            angle += 45;
            fl = robot.frontLeft.getCurrentPosition() + (int)(COUNTS_PER_INCH * Math.cos(Math.toRadians(angle)) * inches);
            fr = robot.frontRight.getCurrentPosition() + (int)(COUNTS_PER_INCH * Math.sin(Math.toRadians(angle)) * inches);
            bl = robot.backLeft.getCurrentPosition() + (int)(COUNTS_PER_INCH * Math.sin(Math.toRadians(angle)) * inches);
            br = robot.backRight.getCurrentPosition() + (int)(COUNTS_PER_INCH * Math.cos(Math.toRadians(angle)) * inches);

            telemetry.addData("Path","fl %d :: br %d :: fr %d :: bl %d",fl,br,fr,bl);
            telemetry.update();

            robot.frontLeft.setTargetPosition(fl);
            robot.frontRight.setTargetPosition(fr);
            robot.backLeft.setTargetPosition(bl);
            robot.backRight.setTargetPosition(br);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.frontLeft.setPower(speed * Math.abs(Math.cos(Math.toRadians(angle))));
            robot.frontRight.setPower(speed * Math.abs(Math.sin(Math.toRadians(angle))));
            robot.backLeft.setPower(speed * Math.abs(Math.sin(Math.toRadians(angle))));
            robot.backRight.setPower(speed * Math.abs(Math.cos(Math.toRadians(angle))));

            while (opModeIsActive() && runtime.seconds() < timeout) {
                int frontEnc = Math.abs(robot.frontLeft.getCurrentPosition());
                int backEnc = Math.abs(robot.backLeft.getCurrentPosition());
                int compareEnc = Math.max(frontEnc, backEnc);

                telemetry.addData("Path","fl %d :: br %d :: fr %d :: bl %d",fl,br,fr,bl);
                telemetry.addData("Position","fl %d :: br %d :: fr %d :: bl %d",
                        robot.frontLeft.getCurrentPosition(), robot.backRight.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition());
                telemetry.addData("Timeout","%.2f : %.2f",timeout,runtime.seconds());
                telemetry.update();
                if (compareEnc > Math.max(Math.abs(fl), Math.abs(bl))) {
                    break;
                }
            }

            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void rotateInPlace (int angle) {
        rotateInPlace(angle, TURN_SPEED);
    }
    public void rotateInPlace (int angle, double speed) {
        int left, right;
        double timeout = angle / 45;
        resetEncoders();
        if(opModeIsActive()) {
            left = (int)(1.56 * COUNTS_PER_INCH * ROBOT_RADIUS * Math.toRadians(angle));
            right = -(int)(1.56 * COUNTS_PER_INCH * ROBOT_RADIUS * Math.toRadians(angle));

            telemetry.addData("Path","left %d :: right %d",left,right);
            telemetry.update();

            robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() + left);
            robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() + left);
            robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() + right);
            robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + right);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.frontLeft.setPower(speed);
            robot.backLeft.setPower(speed);
            robot.frontRight.setPower(speed);
            robot.backRight.setPower(speed);

            while(opModeIsActive() && runtime.seconds() < timeout
                    && robot.frontLeft.isBusy() && robot.backLeft.isBusy()) {
/*
                int frontEnc = Math.abs(robot.frontLeft.getCurrentPosition());
                int backEnc = Math.abs(robot.backLeft.getCurrentPosition());
                int compareEnc = Math.max(frontEnc, backEnc);
*/
                telemetry.addData("Path","left %d :: right %d",left,right);
                telemetry.addData("Position","fl %d :: bl %d :: fr %d :: br %d",
                        robot.frontLeft.getCurrentPosition(), robot.backLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition(), robot.backRight.getCurrentPosition());
                telemetry.addData("Timeout","%.2f : %.2f",timeout,runtime.seconds());
                telemetry.update();
                /*if (compareEnc > Math.abs(left)) {
                    break;
                }*/
            }

            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void launchBall() {
        robot.kickerMotor.setTargetPosition(robot.kickerMotor.getCurrentPosition() +
                (int)(COUNTS_PER_MOTOR_REV * 3 / 2));
        robot.kickerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.kickerMotor.setPower(1.0);
        while(robot.kickerMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Status","Kicking");
            telemetry.update();
        }
        robot.kickerMotor.setPower(0.0);
        robot.kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void nextBall() {
        robot.elevatorMotor.setTargetPosition(robot.kickerMotor.getCurrentPosition() +
                (int)(COUNTS_PER_MOTOR_REV * 3 / 2));
        robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elevatorMotor.setPower(1.0);
        while(robot.elevatorMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Status","Kicking");
            telemetry.update();
        }
        robot.elevatorMotor.setPower(0.0);
        robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetEncoders() {

        telemetry.addData("Status", "Please Wait: Resetting Encoders");    //
        telemetry.update();

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}