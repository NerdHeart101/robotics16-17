package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Jonathon on 12/17/2016.
 */

/**
 * This is a class which contains all of the methods required to make autonomous periods work
 * It contains the encoderDrive method, created by First, along with the code that starts each
 * Autonomous method. This class will be extended by each of the individual autonomous periods
 * in order to avoid code repetition, and in order to increase readability of the methods for each
 * autonomous period.
 *
 * Note that previous methods still work as intended. This is primarily for storing methods used by
 * finalized OpModes.
 */

public class AutonomousBase extends LinearOpMode {

    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.HardwarePushbot robot   = new HardwarePushbot();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440  ;   // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0   ;     // This is < 1.0 if geared UP


    // Competition bot
    static final double     WHEEL_DIAMETER_INCHES   = 6.0   ;     // For figuring circumference
    static final double     DISTANCE_BETWEEN_WHEELS = 16.0  ;    // For figuring bot rotations

    /*
    // Pushbot
    static final double     WHEEL_DIAMETER_INCHES   = 4.0   ;   // For figuring circumference
    static final double     DISTANCE_BETWEEN_WHEELS = 14.625;   // For figuring bot rotations
     */

    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                                                      / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.7   ;
    static final double     TURN_SPEED              = 0.5   ;

    @Override
    public void runOpMode() {
        telemetry.addData("Error", "This should not be seen. Check the AutonomousBase class");
        telemetry.update();
    }

    /*
     *  Method to initialize the robot, using code defined in the original autonomous methods
     *  created by First.
     */
    public void initializeAutonomous() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            double leftRatio = Math.abs(leftInches) / Math.max(Math.abs(leftInches),Math.abs(rightInches));
            double rightRatio = Math.abs(rightInches) / Math.max(Math.abs(leftInches),Math.abs(rightInches));
            double leftSpeed = Math.abs(speed) * leftRatio;
            double rightSpeed = Math.abs(speed) * rightRatio;
            robot.leftMotor.setPower(leftSpeed);
            robot.rightMotor.setPower(rightSpeed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    // A method in order to use the kicker to launch a ball.
    public void launchBall() {

        int newKickerTarget;

        // Make sure opmode is still active
        if(opModeIsActive()) {

            // Get the kicker's position and set the target one position ahead
            newKickerTarget = robot.kickerMotor.getCurrentPosition() + (int)COUNTS_PER_MOTOR_REV;
            robot.kickerMotor.setTargetPosition(newKickerTarget);

            // Set the motor mode and start the motor moving
            robot.kickerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.kickerMotor.setPower(1);

            // As long as the OpMode is active, keep the shooting going
            while(opModeIsActive() && robot.kickerMotor.isBusy()) {

                // Display information for the driver.
                //telemetry.addData("Kicker running:", true);
                //telemetry.update();
                telemetry.addData("Path1", "Kicker running to %7d", newKickerTarget);
                telemetry.addData("Path2", "Kicker at %7d", robot.kickerMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop motion and reset motor run mode
            robot.kickerMotor.setPower(0);
            robot.kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // The following methods are all created in order to further increase readability.
    // And to reduce keystrokes. Efficiency, yo!

    public void driveStraight(double inches) {

        double straightTimeout = Math.abs(inches) / 8.0;
        encoderDrive(DRIVE_SPEED, inches, inches, straightTimeout);

    }

    // A method to rotate the bot in place.
    // Positive degrees is left, negative is right.
    public void rotateInPlace(double degrees) {

        degrees += 5;   // 5 degrees are added to account for errors in the encoders

        double arc = DISTANCE_BETWEEN_WHEELS * degrees * Math.PI / 360;
        double rightArc = -arc;
        double leftArc = arc;
        double rotateTimeout = Math.abs(arc)*TURN_SPEED*2;
        encoderDrive(TURN_SPEED, rightArc, leftArc, rotateTimeout);


    }

    // A method to have the bot move in a circular path to a target
    public void moveToTarget(double distanceToTarget, double degreesToTarget) {

        degreesToTarget += 5;

        double radiansToTarget = Math.toRadians(degreesToTarget);
        double radiansAround = Math.PI - ( 2 * radiansToTarget );
        double turnRadius = distanceToTarget / ( 2 * Math.abs ( Math.cos ( radiansToTarget )));
        double leftTurnRadius = turnRadius - ( DISTANCE_BETWEEN_WHEELS / 2 );
        double rightTurnRadius = turnRadius + ( DISTANCE_BETWEEN_WHEELS / 2 );

        double leftArc = leftTurnRadius * radiansAround;
        double rightArc = rightTurnRadius * radiansAround;

        double moveTimeout = Math.max(leftArc,rightArc) * DRIVE_SPEED * 2;

        // If this does not work, perhaps some changes to the encoder drive method are required.
        // We'll see, won't we?
        encoderDrive(DRIVE_SPEED, leftArc, rightArc, moveTimeout);

    }
}
