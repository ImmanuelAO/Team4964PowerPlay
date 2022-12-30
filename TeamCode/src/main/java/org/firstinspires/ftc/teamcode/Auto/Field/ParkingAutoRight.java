package org.firstinspires.ftc.teamcode.Auto.Field;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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


        ACTI();

        ACTII();

        ACTIII();

        ACTII();


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

    void ACTI(){
        Bot.Claw.setTargetPosition(var.claw_cone);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Claw.setPower(1);
        sleep(5);
        Bot.strafeDrive(-64,.8, this);
        sleep(5);
        Bot.driveStraight(123,.74123456789, this);
        sleep(5);
        Bot.strafeDrive(29.5f,.7,this);
        sleep(5);
        Bot.strafeDrive(8,.5,this);
        sleep(5);
        Bot.gyroTurn(.5,182,this);
    }

    void ACTII(){
        telemetry.addLine(String.valueOf(Bot.Gyro.getHeading()));
        telemetry.update();
        Bot.Lift.setTargetPosition(var.Lvl_Tall);
        sleep(1);
        Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Lift.setPower(1);
        sleep(1250);
        Bot.driveStraight(11, .3, this);
        sleep(1);
        Bot.Claw.setTargetPosition(var.claw_cone);
        Bot.Lift.setTargetPosition(var.Lvl_Tall + 500);
        Bot.Lift.setPower(.7);
        Bot.Claw.setTargetPosition(var.claw_cone);
        sleep(25);
        //Bot.strafeDrive(3,.5,this);
        Bot.Claw.setTargetPosition(var.claw_zero);
        sleep(55);
        Bot.driveStraight(-10, .3, this);
        Bot.Lift.setTargetPosition(-400);
        sleep(55);

    }

    void ACTIII(){
        Bot.gyroTurn(.5, 265, this);
        sleep(5);
        Bot.driveStraight(80,.8,this);
        sleep(5);
        Bot.gyroTurn(.5, 265, this);
        Bot.driveStraight(15,.6,this);
        sleep(5);
        Bot.Claw.setTargetPosition(var.claw_cone);
        Bot.Claw.setPower(5);
        sleep(55);
        Bot.Lift.setTargetPosition(var.Lvl_Tall);
        Bot.Lift.setPower(.65);
        sleep(50);
        Bot.driveStraight(-85,.6,this);
        sleep(5);
        Bot.driveStraight(-11,.6,this);
        sleep(5);
        Bot.gyroTurn(.5, 180,this);
        sleep(5);
        Bot.driveStraight(5,.7,this);
    }

    void ACTIV(){
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
