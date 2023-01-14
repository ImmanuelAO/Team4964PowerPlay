package org.firstinspires.ftc.teamcode.Auto.Field;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Bot;
import org.firstinspires.ftc.teamcode.Robot.Variables;


@Autonomous(name= "Left Parking Auto!!!!!")
public class ParkingAutoLeft extends LinearOpMode {

    Bot robot = new Bot();
    Variables var = new Variables();
    ObjectDetector.POSITIONS pos;
    boolean skip = false;

    @Override
    public void runOpMode() throws InterruptedException {
        ObjectDetector detector = new ObjectDetector(this, true,false);

        robot.init(hardwareMap, this);

        Bot.bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bot.tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bot.bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bot.tRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bot.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bot.Claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        waitForStart();

        // camera decision
        ObjectDetector.POSITIONS position = detector.getDecision(this);
        pos = position;
        telemetry.addData("position ", detector.getDecision(this));



        //robot.strafeDrive(-40, 0, 0.7, this);
        //robot.strafeDrive(0,70, 0.7, this);
        // robot.strafeDrive(-39, 0, 0.7, this);


        REHEARSAL();


//
//
        //    // getting into position to drop cone
        //    //robot.strafeDrive(0, 4, 0.7, this);
        //    Bot.driveStraight(.7, 4,4,4,4,this);
//
        //    Bot.Claw.setTargetPosition(var.claw_open);
        //    //robot.strafeDrive(0, -4, 0.7, this);
        //    Bot.driveStraight(.7, -4,-4,-4,-4,this);
        //    Bot.Lift.setTargetPosition(var.Lvl_Ground);
        //    Bot.Claw.setTargetPosition(var.claw_zero);
        //    //robot.strafeDrive(30, 0, 0.7, this);
        //    //robot.strafeDrive(0, 65, 0.7, this);
        //    Bot.strafeDrive(30,.7, this);
        //    Bot.driveStraight(.7, 65,65,65,65,this);
//
//
        //    // make the decision
        //    switch (position) {
        //        case POS1:
        //         break;
        //         case POS2:
        //             //robot.strafeDrive(55, 0, 0.7, this);
        //             Bot.strafeDrive(55,.7, this);
//
        //             break;
        //             case POS3:
        //                 //robot.strafeDrive(112, 0, 0.7, this);
        //                 Bot.strafeDrive(112,.7, this);
//
        //    }
    }

    void REHEARSAL(){
        ACTI();

        ACTII();

        ACTIII();

        ACTII();
    }

    void BROADWAY(){
        ACTI();

        ACTII();

        ACTIII();

        ACTII();

        ACTV();
    }

    void ACTI(){
        Bot.Claw.setTargetPosition(var.claw_cone);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Claw.setPower(1);
        Bot.gyroTurn(.5,180, this);
        sleep(5);
        Bot.strafeDrive(40,1,this);
        sleep(5);
        Bot.gyroTurn(.5,180,this);
        sleep(5);
        Bot.strafeDrive(36,.7,this);
        sleep(5);
        Bot.gyroTurn(.5,180,this);
        sleep(5);
        Bot.gyroTurn(.5,180,this);
        sleep(5);
        Bot.driveStraight(123,1, 180, this);
        sleep(5);
        Bot.gyroTurn(.5,180,this);
        sleep(5);
        Bot.Lift.setTargetPosition(var.Lvl_Tall);
        sleep(1);
        Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Lift.setPower(.75);
        Bot.strafeDrive(20,.8,this);
        sleep(5);

        int i = 0;
        if(Bot.distance.getDistance(DistanceUnit.CM) <= 40 ) {
            while (Bot.distance.getDistance(DistanceUnit.CM) <= 40){
                telemetry.addLine("stopped for other actor");
                i += 8;
                if(i > 50000){
                    skip = true;
                }
            }
        }

        Bot.strafeDrive(10,.5,this);
        sleep(5);
        if(Bot.distance.getDistance(DistanceUnit.CM) <= 40 ) {
            Bot.driveStraight(-10,.7,180,this);
            while (Bot.distance.getDistance(DistanceUnit.CM) <= 50);
            Bot.driveStraight(10,.7,180,this);
            sleep(5);
        }

        Bot.SensorStrafeDrive(13,.5,30,this);
        Bot.gyroTurn(.5,180,this);
        sleep(5);

    }

    void ACTII(){
        Bot.Lift.setPower(1);
        Bot.gyroTurn(.5,180,this);
        sleep(5);
        correction();
        Bot.driveStraight(14, .5, 180, this);
        sleep(5);
        Bot.Lift.setPower(.25);
        Bot.Claw.setPower(.9);
        Bot.gyroTurn(.5,180,this);
        Bot.Lift.setTargetPosition(var.Lvl_Mid);
        sleep(5);
        Bot.Claw.setPower(.8);
        Bot.Lift.setPower(1);
        Bot.gyroTurn(.5,180,this);
        Bot.Claw.setPower(.6);
        Bot.Lift.setPower(1);
        //Bot.strafeDrive(3,.5,this);
        Bot.Claw.setTargetPosition(var.claw_zero);
        sleep(55);
        Bot.gyroTurn(.5,180,this);
        sleep(5);
        Bot.Claw.setTargetPosition(var.claw_zero);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Claw.setPower(1);
        sleep(55);
        Bot.driveStraight(-14, .3, 180, this);
        Bot.Lift.setTargetPosition(-460);
        sleep(5);
    }

    void ACTIII(){
        Bot.gyroTurn(.5,180,this);
        sleep(5);
        Bot.gyroTurn(.5, 90, this);
        sleep(5);
        Bot.strafeDrive(7,.8,this);
        sleep(5);
        Bot.distance.getDistance(DistanceUnit.CM);
        Bot.driveStraight(92,.8, 90, this);
        sleep(5);
        Bot.distance.getDistance(DistanceUnit.CM);
        Bot.sensorDriveStraight((float) Bot.distance.getDistance(DistanceUnit.CM) - 1f,.3,1, this);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bot.Claw.setPower(5);
        sleep(275);
        Bot.Lift.setTargetPosition(var.Lvl_Short);
        sleep(275);
        Bot.Lift.setTargetPosition(var.Lvl_Tall);
        Bot.Lift.setPower(.5);
        Bot.sensorDriveStraight(-90,.8,90,this);
        sleep(5);
        Bot.Lift.setPower(1);
        Bot.strafeDrive(3,.7,this);
    }

    void correction(){
        Bot.distance.getDistance(DistanceUnit.CM);
        Bot.SensorStrafeDrive(-5,.5,30, this);
        sleep(5);
        Bot.distance.getDistance(DistanceUnit.CM);
        Bot.SensorStrafeDrive(5,.5,30, this);
        sleep(5);
    }

    void ACTV(){
        Bot.gyroTurn(.5,180,this);
        switch (pos) {
            case POS1:
                Bot.strafeDrive(35,.9,this);
                break;
            case POS2:
                Bot.strafeDrive(-30,.9,this);
                break;
            case POS3:
                Bot.strafeDrive(-95,.9,this);
        }
    }
}
