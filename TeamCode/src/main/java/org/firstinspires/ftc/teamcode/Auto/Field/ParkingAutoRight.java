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
    boolean skip = false;
    public float dis = 0;

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

        if(skip){
            ACTV();
        }

        if(!skip) {
            ACTII(90);

            ACTIII(.7);

            ACTII(270);
        }
    }

    void BROADWAY(){

        ACTI();

        if(!skip) {
            ACTII(90);

            ACTIII(.9);

            ACTII(270);
        }

        ACTV();
    }

    void ACTI(){
        Bot.gyroTurn(.5,180, this);
        sleep(5);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bot.Claw.setPower(-4);
        Bot.strafeDrive(10,1,180,this);
        sleep(5);
        Bot.driveStraight(10,1, 185, this);
        sleep(5);
        Bot.gyroTurn(.5,90,this);
        sleep(5);
        Bot.strafeDrive(180, .7,90,this);
        sleep(5);
        Bot.Lift.setTargetPosition(var.Lvl_Mid);
        sleep(1);
        Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Lift.setPower(.75);
        Bot.SensorStrafeDrive(10,.5,15,this);
        sleep(500);
        dis = (float)Bot.distance.getDistance(DistanceUnit.CM);
        Bot.Lift.setTargetPosition(var.Lvl_Tall);
        sleep(250);
    }

    void ACTII(int angle){
        Bot.Lift.setPower(5);
        Bot.gyroTurn(.5,angle,this);
        sleep(5);
        dis = (float)Bot.distance.getDistance(DistanceUnit.CM);
        sleep(5);
        Bot.driveStraight(dis - 1, .5, angle, this);
        sleep(5);
        Bot.Lift.setPower(.25);
        Bot.Claw.setPower(.9);
        Bot.gyroTurn(.5,angle,this);
        Bot.Lift.setTargetPosition(var.Lvl_Mid);
        sleep(5);
        Bot.Claw.setPower(.8);
        Bot.Lift.setPower(1);
        Bot.gyroTurn(.5,angle,this);
        Bot.Claw.setPower(.6);
        Bot.Lift.setPower(1);
        //Bot.strafeDrive(3,.5,this);
        sleep(55);
        Bot.gyroTurn(.5,angle,this);
        sleep(5);
        Bot.Claw.setTargetPosition(var.claw_zero);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Claw.setPower(1);
        sleep(55);
        Bot.driveStraight(-14, .3, angle, this);
        Bot.Lift.setTargetPosition(-460);
        sleep(5);
    }

    void ACTIII(double urgency){
        Bot.gyroTurn(.5,90,this);
        sleep(5);
        Bot.gyroTurn(.5, 270, this);
        sleep(5);
        Bot.strafeDrive(-50,.8,270,this);
        sleep(5);
        Bot.distance.getDistance(DistanceUnit.CM);
        Bot.driveStraight(92,.8, 270, this);
        sleep(5);
        double i = Bot.distance.getDistance(DistanceUnit.CM);
        correction(12);
        while(Bot.distance.getDistance(DistanceUnit.CM) > i){
            Bot.Lift.setTargetPosition((int) (Bot.Lift.getCurrentPosition() + urgency * 2));
        }
        Bot.Lift.setTargetPosition(Bot.Lift.getCurrentPosition() - 50);
        Bot.sensorDriveStraight((float) Bot.distance.getDistance(DistanceUnit.CM) - 1f,.3,1, this);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bot.Claw.setPower(5);
        sleep(275);
        Bot.Lift.setTargetPosition(var.Lvl_Short);
        sleep(275);
        Bot.Lift.setTargetPosition(var.Lvl_Mid);
        Bot.Lift.setPower(.5);
        Bot.driveStraight(-120,.8,270,this);
        sleep(5);
        Bot.Lift.setPower(1);
        Bot.strafeDrive(50,.7,270,this);
        sleep(450);
        dis = (float)Bot.distance.getDistance(DistanceUnit.CM);
        Bot.Lift.setTargetPosition(var.Lvl_Tall);
        sleep(250);
    }

    void correction(int distance){
        Bot.distance.getDistance(DistanceUnit.CM);
        Bot.SensorStrafeDrive(-5,.5,distance, this);
        sleep(5);
        Bot.distance.getDistance(DistanceUnit.CM);
        Bot.SensorStrafeDrive(5,.5,-distance, this);
        sleep(5);
    }

    void ACTV(){
        Bot.gyroTurn(.5,180,this);
        switch (pos) {
            case POS1:
                Bot.strafeDrive(-35,.9,180,this);
                break;
            case POS2:
                Bot.strafeDrive(30,.9,180,this);
                break;
            case POS3:
                Bot.strafeDrive(95,.9,180,this);
        }
    }
}
