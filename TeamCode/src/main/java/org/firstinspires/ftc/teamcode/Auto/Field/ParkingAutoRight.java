package org.firstinspires.ftc.teamcode.Auto.Field;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Bot;
import org.firstinspires.ftc.teamcode.Robot.Variables;


@Autonomous(name= "Right Parking Auto!!!!!")
public class ParkingAutoRight extends LinearOpMode {

    Bot robot = new Bot();
    Variables var = new Variables();
    ObjectDetector.POSITIONS pos;

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
        Bot.strafeDrive(-30,1,this);
        sleep(5);
        Bot.gyroTurn(.5,180,this);
        sleep(5);
        Bot.strafeDrive(-36,.7,this);
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
            while (Bot.distance.getDistance(DistanceUnit.CM) <= 40 || i > 50000){
                telemetry.addLine("stopped for other actor");
                i += 8;
            }
        }


        Bot.SensorStrafeDrive(22,.5,30,this);
        sleep(5);
        Bot.gyroTurn(.5,180,this);
        sleep(5);

    }

    void ACTII(){
        Bot.Lift.setPower(1);
        Bot.Claw.setTargetPosition(var.claw_zero);
        Bot.Claw.setPower(.25);
        Bot.gyroTurn(.5,180,this);
        sleep(5);
        correction();
        Bot.Claw.setTargetPosition(var.claw_zero);
        Bot.Claw.setPower(.5);
        Bot.driveStraight(14, .5, 180, this);
        sleep(5);
        Bot.Lift.setPower(.25);
        Bot.Claw.setTargetPosition(var.claw_zero);
        Bot.Claw.setPower(.9);
        Bot.gyroTurn(.5,180,this);
        Bot.Lift.setTargetPosition(var.Lvl_Mid);
        sleep(5);
        Bot.Claw.setTargetPosition(var.claw_zero);
        Bot.Claw.setPower(.8);
        Bot.Lift.setPower(1);
        Bot.Claw.setTargetPosition(var.claw_zero);
        sleep(600);
        Bot.gyroTurn(.5,180,this);
        Bot.Claw.setTargetPosition(var.claw_zero + 20);
        Bot.Claw.setPower(1);
        sleep(750);
        Bot.Lift.setPower(1);
        //Bot.strafeDrive(3,.5,this);
        Bot.Claw.setTargetPosition(var.claw_zero + 10);
        sleep(55);
        Bot.gyroTurn(.5,180,this);
        sleep(5);
        Bot.driveStraight(-14, .3, 180, this);
        Bot.Lift.setTargetPosition(-460);
        Bot.Claw.setTargetPosition(var.claw_cone + 14);
        Bot.Claw.setPower(1);
        sleep(55);
    }

    void ACTIII(){
        Bot.gyroTurn(.5,180,this);
        sleep(5);
        Bot.gyroTurn(.5, 270, this);
        sleep(5);
        Bot.strafeDrive(-7,.8,this);
        sleep(5);
        Bot.distance.getDistance(DistanceUnit.CM);
        Bot.sensorDriveStraight(92,.8, 25, this);
        sleep(5);
        Bot.Claw.setTargetPosition(-10);
        Bot.Claw.setPower(0.75);
        sleep(55);
        Bot.Claw.setTargetPosition(var.claw_cone - 10);
        Bot.Claw.setPower(1);
        sleep(5);
        Bot.distance.getDistance(DistanceUnit.CM);
        Bot.sensorDriveStraight(24,.3,.5, this);
        Bot.Claw.setTargetPosition(var.claw_cone - 17);
        Bot.Claw.setPower(5);
        sleep(275);
        Bot.Lift.setTargetPosition(var.Lvl_Short);
        sleep(275);
        Bot.Lift.setTargetPosition(var.Lvl_Tall);
        Bot.Lift.setPower(.5);
        Bot.driveStraight(-90,.8,270,this);
        sleep(5);
        Bot.Lift.setPower(1);
        Bot.strafeDrive(-3,.7,this);
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
                Bot.strafeDrive(-35,.9,this);
                break;
            case POS2:
                Bot.strafeDrive(30,.9,this);
                break;
            case POS3:
                Bot.strafeDrive(95,.9,this);
        }
    }
}
