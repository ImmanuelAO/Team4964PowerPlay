package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Bot {
    public static DcMotor tLeftDT  = null;
    public static DcMotor bLeftDT  = null;
    public static DcMotor tRightDT = null;
    public static DcMotor bRightDT = null;
    public static DcMotor Lift     = null;
    public static DcMotor Claw     = null;

    public static ModernRoboticsI2cGyro Gyro;
    public static Rev2mDistanceSensor distance;


    static final double HEADING_THRESHOLD = 178;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.07;     // Larger is more responsive, but also less stable

    public static double DRIVE_SPEED = .7;
    public static double TURN_SPEED = 0.4;
    public static double amountError = 2;



    public static final double conversion = Variables.conversion; // var.conversion of encoder rotations to centimetres
    public static final double tileConversion = conversion * 60; // var.conversion of encoder rotations to tiles !!EDIT!!

    public void init(HardwareMap ahwMap, OpMode opMode) {
        opMode.telemetry.addLine("wait for it... ");
        opMode.telemetry.update();

        HardwareMap hwMap = ahwMap;
        Variables var = new Variables();
        tLeftDT   = hwMap.get(DcMotor.class, "FrontL");
        bLeftDT   = hwMap.get(DcMotor.class, "BackL");
        tRightDT  = hwMap.get(DcMotor.class, "FrontR");
        bRightDT  = hwMap.get(DcMotor.class, "BackR");
        Lift      = hwMap.get(DcMotor.class, "lift"    );
        Claw      = hwMap.get(DcMotor.class, "claw"    );
        Gyro      = hwMap.get(ModernRoboticsI2cGyro.class , "gyro");
        distance  = hwMap.get(Rev2mDistanceSensor.class, "distanceSensor");


        tLeftDT.setDirection(DcMotor.Direction.FORWARD);
        tLeftDT.setDirection(DcMotor.Direction.FORWARD);
        bRightDT.setDirection(DcMotor.Direction.REVERSE);
        bLeftDT.setDirection(DcMotor.Direction.FORWARD);
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Claw.setDirection(DcMotor.Direction.FORWARD);


        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Gyro.calibrate();

        tRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bLeftDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tLeftDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tRightDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tRightDT.setPower(0);
        tLeftDT.setPower(0);
        bRightDT.setPower(0);
        bLeftDT.setPower(0);
        Lift.setPower(0);
        Claw.setPower(0);


        opMode.telemetry.addLine("Initialization Complete! ;) ");
        opMode.telemetry.update();

    }

    public static void liftSet (int height, double speed, LinearOpMode opMode){
        boolean done = false;

        Lift.setTargetPosition((int) (height * 1.1));

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        Lift.setPower(speed);

        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }
    public static void SensorStrafeDrive (float distance, double speed, double sensorDistance, LinearOpMode opMode)
    {
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // if it breaks do this https://github.com/AnishJag/FTCFreightFrenzy/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Base/MainBase.java
        if(opMode.opModeIsActive()) {
            double error = distance / Math.abs(distance);

            boolean done = false;

            int tLeftPower = tLeftDT.getCurrentPosition() + (int) (conversion * -distance * 1.1 - (error * 1.5 * speed));
            int bLeftPower = bLeftDT.getCurrentPosition() + (int) (conversion * distance * 1.1 - (error * 1.5 * speed));
            int tRightPower = tRightDT.getCurrentPosition() + (int) (conversion * distance * 1.1 + (error * 1.5 * speed));
            int bRightPower = bRightDT.getCurrentPosition() + (int) (conversion * -distance * 1.1 + (error * 1.5 * speed));

            tLeftDT.setTargetPosition(tLeftPower);
            bLeftDT.setTargetPosition(bLeftPower);
            tRightDT.setTargetPosition(tRightPower);
            bRightDT.setTargetPosition(bRightPower);

            tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            tLeftDT.setPower(speed);
            tRightDT.setPower(speed);
            bLeftDT.setPower(speed);
            bRightDT.setPower(speed);

            while (opMode.opModeIsActive() && !done) {

                double actError = error * (Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition())) + error *  (Math.abs(bLeftPower) - Math.abs(bLeftDT.getCurrentPosition())) + error *
                        (Math.abs(tRightPower) - Math.abs(tRightDT.getCurrentPosition())) + error *  (Math.abs(bRightPower) - Math.abs(bRightDT.getCurrentPosition()));
                double currentDistance = Bot.distance.getDistance(DistanceUnit.CM);

                if ((error >= actError - .7 && error <= actError + .7) || currentDistance < sensorDistance) {
                    done = true;
                    tLeftDT.setPower(0);
                    tRightDT.setPower(0);
                    bLeftDT.setPower(0);
                    bRightDT.setPower(0);
                }

                else if ( Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition()) < Math.abs(tLeftPower) / distance * conversion - 10) {
                    tLeftDT.setPower(speed - 0.95);
                    tRightDT.setPower(speed - 0.95);
                    bLeftDT.setPower(speed - 0.95);
                    bRightDT.setPower(speed - 0.85);
                }

                opMode.telemetry.addLine("distance:" + Bot.distance.getDistance(DistanceUnit.CM));
                opMode.telemetry.update();
            }


        }

    }

    public static void sensorDriveStraight (float distance, double speed, double range, LinearOpMode opMode)
    {
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.telemetry.update();
        // if it breaks do this https://github.com/AnishJag/FTCFreightFrenzy/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Base/MainBase.java
        if(opMode.opModeIsActive()) {
            double error = distance / Math.abs(distance);


            boolean done = false;

            int tLeftPower = tLeftDT.getCurrentPosition() - (int) (conversion * distance * 1.1 + (error * 1.5 * speed));
            int bLeftPower = bLeftDT.getCurrentPosition() - (int) (conversion * distance * 1.1 + (error * 1.5 * speed));
            int tRightPower = tRightDT.getCurrentPosition() - (int) (conversion * distance * 1.1 - (error * 1.5 * speed));
            int bRightPower = bRightDT.getCurrentPosition() - (int) (conversion * distance * 1.1 - (error * 1.5 * speed));

            tLeftDT.setTargetPosition(tLeftPower);
            bLeftDT.setTargetPosition(bLeftPower);
            tRightDT.setTargetPosition(tRightPower);
            bRightDT.setTargetPosition(bRightPower);

            tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.1, speed + 0.15);
            tLeftDT.setPower(speed);
            tRightDT.setPower(speed);
            bLeftDT.setPower(speed);
            bRightDT.setPower(speed);

            while (opMode.opModeIsActive() && !done) {

                double actError = error * (Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition())) + error *  (Math.abs(bLeftPower) - Math.abs(bLeftDT.getCurrentPosition())) + error *
                        (Math.abs(tRightPower) - Math.abs(tRightDT.getCurrentPosition())) + error *  (Math.abs(bRightPower) - Math.abs(bRightDT.getCurrentPosition()));
                double currentDistance = Bot.distance.getDistance(DistanceUnit.CM);

                if ((error >= actError - .7 && error <= actError + .7) || currentDistance < range) {
                    done = true;
                    tLeftDT.setPower(0);
                    tRightDT.setPower(0);
                    bLeftDT.setPower(0);
                    bRightDT.setPower(0);
                }

                else if ( Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition()) < Math.abs(tLeftPower) / distance * conversion - 10) {
                    tLeftDT.setPower(speed - 0.95);
                    tRightDT.setPower(speed - 0.95);
                    bLeftDT.setPower(speed - 0.95);
                    bRightDT.setPower(speed - 0.85);
                }

                opMode.telemetry.addLine("distance:" + Bot.distance.getDistance(DistanceUnit.CM));
                opMode.telemetry.update();
            }





        }

    }



    //driving using only Mecanum strafe
    public static void strafeDrive (float distance, double speed, LinearOpMode opMode)
    {
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.telemetry.update();
        // if it breaks do this https://github.com/AnishJag/FTCFreightFrenzy/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Base/MainBase.java
        if(opMode.opModeIsActive()) {
            double error = distance / Math.abs(distance);

            boolean done = false;

            int tLeftPower = tLeftDT.getCurrentPosition() + (int) (conversion * -distance * 1.1 - (error * 1.5 * speed));
            int bLeftPower = bLeftDT.getCurrentPosition() + (int) (conversion * distance * 1.1 - (error * 1.5 * speed));
            int tRightPower = tRightDT.getCurrentPosition() + (int) (conversion * distance * 1.1 + (error * 1.5 * speed));
            int bRightPower = bRightDT.getCurrentPosition() + (int) (conversion * -distance * 1.1 + (error * 1.5 * speed));

            tLeftDT.setTargetPosition(tLeftPower);
            bLeftDT.setTargetPosition(bLeftPower);
            tRightDT.setTargetPosition(tRightPower);
            bRightDT.setTargetPosition(bRightPower);

            tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.1, speed + 0.15);
            tLeftDT.setPower(speed - 0.1);
            tRightDT.setPower(speed - 0.1);
            bLeftDT.setPower(speed - 0.1);
            bRightDT.setPower(speed - 0.1);

            while (opMode.opModeIsActive() && !done) {
                speed = Range.clip(Math.abs(speed), 0.05, speed + 0.15);

                double actError = error * (Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition())) + error *  (Math.abs(bLeftPower) - Math.abs(bLeftDT.getCurrentPosition())) + error *
                        (Math.abs(tRightPower) - Math.abs(tRightDT.getCurrentPosition())) + error *  (Math.abs(bRightPower) - Math.abs(bRightDT.getCurrentPosition()));

                if ( error >= actError - 1 && error <= actError + 1) {
                    tLeftDT.setPower(0);
                    tRightDT.setPower(0);
                    bLeftDT.setPower(0);
                    bRightDT.setPower(0);
                    done = true;
                }

                else if( Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition()) > Math.abs(tLeftPower) / 2) {
                    tLeftDT.setPower(speed + 0.025);
                    tRightDT.setPower(speed + 0.025);
                    bLeftDT.setPower(speed + 0.025);
                    bRightDT.setPower(speed + 0.025);
                }

                else if( Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition()) < Math.abs(tLeftPower) / 2) {
                    tLeftDT.setPower(speed - 0.025);
                    tRightDT.setPower(speed - 0.025);
                    bLeftDT.setPower(speed - 0.025);
                    bRightDT.setPower(speed - 0.025);
                }

                else if ( Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition()) < Math.abs(tLeftPower) / distance * conversion - 10) {
                    tLeftDT.setPower(speed - 0.95);
                    tRightDT.setPower(speed - 0.95);
                    bLeftDT.setPower(speed - 0.95);
                    bRightDT.setPower(speed - 0.85);
                }
            }


        }

    }

    public static void driveStraight (float distance, double speed, double angle, LinearOpMode opMode)
    {
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.telemetry.update();
        // if it breaks do this https://github.com/AnishJag/FTCFreightFrenzy/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Base/MainBase.java
        if(opMode.opModeIsActive()) {
            double error = distance / Math.abs(distance);


            boolean done = false;

            int tLeftPower = tLeftDT.getCurrentPosition() - (int) (conversion * distance * 1.1 + (error * 1.5 * speed));
            int bLeftPower = bLeftDT.getCurrentPosition() - (int) (conversion * distance * 1.1 + (error * 1.5 * speed));
            int tRightPower = tRightDT.getCurrentPosition() - (int) (conversion * distance * 1.1 - (error * 1.5 * speed));
            int bRightPower = bRightDT.getCurrentPosition() - (int) (conversion * distance * 1.1 - (error * 1.5 * speed));

            tLeftDT.setTargetPosition(tLeftPower);
            bLeftDT.setTargetPosition(bLeftPower);
            tRightDT.setTargetPosition(tRightPower);
            bRightDT.setTargetPosition(bRightPower);

            tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.1, speed + 0.15);
            tLeftDT.setPower(speed);
            tRightDT.setPower(speed);
            bLeftDT.setPower(speed);
            bRightDT.setPower(speed);

            while (opMode.opModeIsActive() && !done) {
                speed = Range.clip(Math.abs(speed), 0.05, speed + 0.15);

                double actError = error * (Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition())) + error *  (Math.abs(bLeftPower) - Math.abs(bLeftDT.getCurrentPosition())) + error *
                        (Math.abs(tRightPower) - Math.abs(tRightDT.getCurrentPosition())) + error *  (Math.abs(bRightPower) - Math.abs(bRightDT.getCurrentPosition()));

                if ((error >= actError - 1 && error <= actError + 1) && onHeading(speed - 0.65, angle, P_TURN_COEFF, opMode)) {
                    tLeftDT.setPower(0);
                    tRightDT.setPower(0);
                    bLeftDT.setPower(0);
                    bRightDT.setPower(0);
                    done = true;
                }

                else if( Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition()) > Math.abs(tLeftPower) / 2) {
                    tLeftDT.setPower(speed + 0.025);
                    tRightDT.setPower(speed + 0.025);
                    bLeftDT.setPower(speed + 0.025);
                    bRightDT.setPower(speed + 0.025);
                }

                else if ( Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition()) < Math.abs(tLeftPower) / 2) {
                    tLeftDT.setPower(speed - 0.025);
                    tRightDT.setPower(speed - 0.025);
                    bLeftDT.setPower(speed - 0.025);
                    bRightDT.setPower(speed - 0.025);
                }

                else if ( Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition()) < Math.abs(tLeftPower) / distance * conversion - 10) {
                    tLeftDT.setPower(speed - 0.95);
                    tRightDT.setPower(speed - 0.95);
                    bLeftDT.setPower(speed - 0.95);
                    bRightDT.setPower(speed - 0.95);
                }
            }





        }

    }

    public static void gyroTurn(double speed, double angle, LinearOpMode opmode) {
        // keep looping while we are still active, and not on heading.
        double deltaAngle = angle - Gyro.getHeading();
        if( angle > Gyro.getHeading() ) {
            while (opmode.opModeIsActive() && !onHeading(speed, deltaAngle, P_TURN_COEFF, opmode)) {
                // Update telemetry & Allow time for other processes to run.
                opmode.telemetry.update();
            }
        }
        else {
            while (opmode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF, opmode)) {
                // Update telemetry & Allow time for other processes to run.
                opmode.telemetry.update();
            }
        }
    }

    static boolean onHeading(double speed, double angle, double PCoeff, LinearOpMode opmode) {
        double   error;
        double   steer;
        boolean  onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(-angle);

        if (Math.abs(error) >= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);

            leftSpeed  = speed * steer;
            rightSpeed   = -leftSpeed;
        }

        tLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send desired speeds to motors.
        tLeftDT.setPower(leftSpeed);
        bLeftDT.setPower(leftSpeed);
        tRightDT.setPower(rightSpeed);
        bRightDT.setPower(rightSpeed);

        // Display it for the driver.
        opmode.telemetry.addData("Target", "%5.2f", angle);
        opmode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opmode.telemetry.addData("Speed ", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        opmode.telemetry.addData("current Angle", Gyro.getHeading());
        opmode.telemetry.addData("current Angle Z", Gyro.getIntegratedZValue());

        return onTarget;
    }

    public static double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - Gyro.getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return -robotError;
    }

    public static double getSteer(double err, double PCoeff) {
        return Range.clip(err * PCoeff, -DRIVE_SPEED, 1);
    }

    public static void gyroDrive(double speed,
                                   double fLeftcm, double fRightcm, double bLeftcm,
                                   double bRightcm,
                                   double angle, LinearOpMode opMode) {

            int newFrontLeftTarget;
            int newFrontRightTarget;
            int newBackLeftTarget;
            int newBackRightTarget;

            double HalfMaxOne;
            double HalfMaxTwo;

            double max;

            double error;
            double steer;
            double frontLeftSpeed;
            double frontRightSpeed;
            double backLeftSpeed;
            double backRightSpeed;

            double ErrorAmount;
            boolean goodEnough = false;



            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = tLeftDT.getCurrentPosition() + (int) (fLeftcm * conversion);
            newFrontRightTarget = tRightDT.getCurrentPosition() + (int) (fRightcm * conversion);
            newBackLeftTarget = bLeftDT.getCurrentPosition() + (int) (bLeftcm * conversion);
            newBackRightTarget = bRightDT.getCurrentPosition() + (int) (bRightcm * conversion);


            // Set Target and Turn On RUN_TO_POSITION
            tLeftDT.setTargetPosition(newFrontLeftTarget);
            tRightDT.setTargetPosition(newFrontRightTarget);
            bLeftDT.setTargetPosition(newBackLeftTarget);
            bRightDT.setTargetPosition(newBackRightTarget);

            tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            tLeftDT.setPower(Math.abs(speed));
            tRightDT.setPower(Math.abs(speed));
            bLeftDT.setPower(Math.abs(speed));
            bRightDT.setPower(Math.abs(speed));
            // keep looping while we are still active, and BOTH motors are running.
            while (((tLeftDT.isBusy() && tRightDT.isBusy()) &&
                    (bLeftDT.isBusy() && bRightDT.isBusy())) && !goodEnough) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (fLeftcm < 0 && fRightcm < 0 && bLeftcm < 0 && bRightcm < 0)
                    steer *= -1.0;

                frontLeftSpeed = speed - steer;
                backLeftSpeed = speed - steer;
                backRightSpeed = speed + steer;
                frontRightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                HalfMaxOne = Math.max(Math.abs(frontLeftSpeed), Math.abs(backLeftSpeed));
                HalfMaxTwo = Math.max(Math.abs(frontRightSpeed), Math.abs(backRightSpeed));
                max = Math.max(Math.abs(HalfMaxOne), Math.abs(HalfMaxTwo));
                if (max > 1.0) {
                    frontLeftSpeed /= max;
                    frontRightSpeed /= max;
                    backLeftSpeed /= max;
                    backRightSpeed /= max;
                }

                tLeftDT.setPower(frontLeftSpeed);
                tRightDT.setPower(frontRightSpeed);
                bLeftDT.setPower(backLeftSpeed);
                bRightDT.setPower(backRightSpeed);

                // Display drive status for the driver.
                opMode.telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                opMode.telemetry.addData("Target", "%7d:%7d", newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                opMode.telemetry.addData("Actual", "%7d:%7d", bLeftDT.getCurrentPosition(), bRightDT.getCurrentPosition(), tLeftDT.getCurrentPosition(), tRightDT.getCurrentPosition());
                opMode.telemetry.addData("Speed", "%5.2f:%5.2f", backLeftSpeed, backRightSpeed, frontLeftSpeed, frontRightSpeed);
                opMode.telemetry.update();

                ErrorAmount = ((Math.abs(((newBackLeftTarget) - (bLeftDT.getCurrentPosition())))
                        + (Math.abs(((newFrontLeftTarget) - (tLeftDT.getCurrentPosition()))))
                        + (Math.abs((newBackRightTarget) - (bRightDT.getCurrentPosition())))
                        + (Math.abs(((newFrontRightTarget) - (tRightDT.getCurrentPosition()))))) / conversion);
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }


            // Stop all motion;
            tLeftDT.setPower(0);
            tRightDT.setPower(0);
            bLeftDT.setPower(0);
            bRightDT.setPower(0);

            // Turn off RUN_TO_POSITION
            tLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            tRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }




    }

    public static void strafeToPosition(double cm, double speed) {
        //
        int move = (int) (Math.round(cm * conversion * 0.8));

        //
         bLeftDT.setTargetPosition( bLeftDT.getCurrentPosition() - move);
         tLeftDT.setTargetPosition( tLeftDT.getCurrentPosition() + move);
         bRightDT.setTargetPosition( bRightDT.getCurrentPosition() + move);
         tRightDT.setTargetPosition( tRightDT.getCurrentPosition() - move);
        //
         tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         tRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         bRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
         tLeftDT.setPower(speed);
         bLeftDT.setPower(speed);
         tRightDT.setPower(speed);
         bRightDT.setPower(speed);
        //
        while ( tLeftDT.isBusy() &&  tRightDT.isBusy() &&  bLeftDT.isBusy() &&  bRightDT.isBusy()) {

        }
         tRightDT.setPower(0);
         tLeftDT.setPower(0);
         bRightDT.setPower(0);
         bLeftDT.setPower(0);

    }




}


