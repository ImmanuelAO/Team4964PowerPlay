package org.firstinspires.ftc.teamcode.Auto.Field;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Bot;
import org.firstinspires.ftc.teamcode.Robot.Variables;


@Autonomous(name= "Right Scoring Auto")
public class ScoringAutoRight extends LinearOpMode {

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

        ObjectDetector.POSITIONS position = detector.getDecision(this);
        pos = position;
        telemetry.addData("position ", detector.getDecision(this));



        ACTI();

        //ACTII();

        //ACTIII();

        //ACTIV();

        //ACTV();


    }

    void ACTI(){
//        Bot.Claw.setTargetPosition(var.claw_cone);
//        sleep(1);
//        Bot.driveStraight(20,.8,this);
//        sleep(1);
//        Bot.gyroTurn(.7,90,this);
//        sleep(1);
//        Bot.strafeDrive(-5,.8,this);
//        Bot.Claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Bot.Claw.setPower(1);
//        sleep(55);
//        Bot.driveStraight(-70,.5,this);
//        sleep(5);

    }

    void ACTII(){
//        Bot.strafeDrive(-115, .6, this);
//        sleep(5);
//        Bot.Lift.setTargetPosition(var.Lvl_Short + 500);
//        Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Bot.Lift.setPower(1);
//        sleep(550);
//        Bot.Claw.setTargetPosition(var.claw_cone);
//        sleep(55);
//        Bot.driveStraight(5,.9,this);
//        sleep(5);
    }

    void ACTIII(){
//        Bot.driveStraight(-75, .5, this);
//        sleep(5);
//        Bot.gyroTurn(1,90,this);
//        sleep(1);
    }

    void ACTIV(){
//        Bot.Lift.setTargetPosition(var.Lvl_Tall);
//        sleep(-var.Lvl_Tall);
//        Bot.driveStraight(13, .3, this);
//        Bot.Lift.setTargetPosition(var.Lvl_Tall + 200);
//        sleep(100);
//        Bot.strafeDrive(3,.5,this);
//        Bot.Claw.setTargetPosition(var.claw_zero);
//        sleep(2000);
//        Bot.driveStraight(-20, .3, this);
//        sleep(1);
//        Bot.Claw.setTargetPosition(var.claw_cone);
//        sleep(750);
//        Bot.Lift.setTargetPosition(var.Lvl_Ground);
//        sleep(-var.Lvl_Tall);
//        Bot.Lift.setPower(0);
    }

    void ACTV(){
//        switch (pos) {
//            case POS1:
//                Bot.strafeDrive(-35,.9,this);
//                break;
//            case POS2:
//                Bot.strafeDrive(30,.9,this);
//                break;
//            case POS3:
//                Bot.strafeDrive(95,.9,this);
//        }
      }
}
