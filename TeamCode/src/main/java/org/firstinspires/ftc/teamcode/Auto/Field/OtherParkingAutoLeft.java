package org.firstinspires.ftc.teamcode.Auto.Field;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Bot;
import org.firstinspires.ftc.teamcode.Robot.Variables;


@Autonomous(name= "Other Left Parking Auto!!!!!!")
public class OtherParkingAutoLeft extends LinearOpMode {

    Bot robot = new Bot();
    Variables var = new Variables();
    ObjectDetector.POSITIONS pos;

    @Override
    public void runOpMode() throws InterruptedException {
        ObjectDetector detector = new ObjectDetector(this, true, false);

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

        ACTI();

        ACTII();

        ACTIII();
    }

    void ACTI() {
        Bot.Claw.setTargetPosition(var.claw_cone);
        sleep(1);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Claw.setPower(1);
        sleep(550);
        Bot.strafeDrive(75, .5, this);
        sleep(5);
        Bot.driveStraight(125, .5, 180,this);
        sleep(5);
        Bot.SensorStrafeDrive(-50, 0.2, 30,this);
        Bot.strafeDrive(-12, .3, this);
    }

    void ACTII() {
        Bot.Lift.setTargetPosition(var.Lvl_Tall);
        sleep(1);
        Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Lift.setPower(1);
        sleep(-var.Lvl_Tall);
        Bot.driveStraight(14, .3, 180,this);
        Bot.Lift.setTargetPosition(var.Lvl_Tall + 600);
        sleep(75);
        //Bot.strafeDrive(3,.5,this);
        Bot.Claw.setTargetPosition(var.claw_zero);
        sleep(2000);
        Bot.driveStraight(-15, .3, 180,this);
        sleep(1);
        Bot.Claw.setTargetPosition(var.claw_cone);
        sleep(1200);
        Bot.Lift.setTargetPosition(var.Lvl_Ground);
        //sleep(-var.Lvl_Tall);
        Bot.Lift.setPower(0);
    }

    void ACTIII() {
        switch (pos) {
            case POS1:
                Bot.strafeDrive(-98, .5, this);
                break;
            case POS2:
                Bot.strafeDrive(-40, .5, this);
                break;
            case POS3:
                Bot.strafeDrive(35, .5, this);
        }
    }
}