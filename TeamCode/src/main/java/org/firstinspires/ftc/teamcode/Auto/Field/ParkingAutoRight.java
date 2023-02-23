package org.firstinspires.ftc.teamcode.Auto.Field;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Bot;
import org.firstinspires.ftc.teamcode.Robot.Variables;


@Autonomous(name= "Right Parking Auto")
public class ParkingAutoRight extends LinearOpMode {

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


        //robot.strafeDrive(-40, 0, 0.7, this);
        //robot.strafeDrive(0,70, 0.7, this);
        // robot.strafeDrive(-39, 0, 0.7, this);


        if (ACTI()) {
            double l = Bot.distance.getDistance(DistanceUnit.CM);
            sleep(5);
            ACTII((int) l - 11);
            ACTIII();
        } else {
            ACTIV();
            ACTV();
        }

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

    boolean ACTI() {
        //close claw to hold cone
        Bot.Claw.setTargetPosition(var.claw_cone);
        sleep(1);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Bot.Claw.setPower(-1);
        sleep(550);
        //strafe to the left
        Bot.strafeDrive(-55, .4, this);
        sleep(5);
        //drive forward
        Bot.driveStraight(125, .6, this);
        sleep(5);
        //begin tracking distance with the sensor
        Bot.distance.getDistance(DistanceUnit.CM);
        int i = 80;
        //safety check for opposing robots in the way
        while (opModeIsActive() && Bot.distance.getDistance(DistanceUnit.CM) < 40 && i > 0) {
            sleep(50);
            i--;
        }
        //commands for when the opposing robot is still in the way
        if (i == 0) {
            return false;
        }
        //commands for when there is no robot blocking the path
        else {
            Bot.SensorStrafeDrive(50, .2, this);
        }
        //int i=0;
//        while(i++<500) {
//            telemetry.addLine("distance:" + Bot.distance.getDistance(DistanceUnit.CM));
//            telemetry.update();
//        }
        //Bot.strafeDrive(-12, 0.3, this);
        return true;
    }

    void ACTII(int distance) {
        //raise the lift to place cone
        Bot.Lift.setTargetPosition(var.Lvl_Tall);
        sleep(1);
        Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Lift.setPower(1);
        sleep(-var.Lvl_Tall);
        sleep(300);
        //Bot.strafeDrive(3,.5,this);
        Bot.driveStraight(distance, 0.5, this);
        //release cone onto junction
        Bot.Claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bot.Claw.setTargetPosition(var.claw_zero);
        sleep(1);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Claw.setPower(1);
        sleep(1000);
        //back away from junction
        Bot.driveStraight(-16, .3, this);
        sleep(200);
        //close claw
        Bot.Claw.setTargetPosition(var.claw_cone);
        sleep(500);
        //lower the lift
        Bot.Lift.setTargetPosition(var.Lvl_Ground);
        sleep(-var.Lvl_Tall);
        Bot.Lift.setPower(0);
    }

    void ACTIII(){
        switch (pos) {
            case POS1:
                Bot.strafeDrive(98,.5,this);
                break;
            case POS2:
                Bot.strafeDrive(40,.5,this);
                break;
            case POS3:
                Bot.strafeDrive(-35,.5,this);
        }
    }


    void ACTIV() {
        Bot.driveStraight(-57, .5, this);
        Bot.Lift.setTargetPosition(var.Lvl_Mid);
        sleep(1);
        Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Lift.setPower(1);
        Bot.distance.getDistance(DistanceUnit.CM);
        Bot.SensorStrafeDrive(-60, .3, this);
        sleep(5);
        double l = Bot.distance.getDistance(DistanceUnit.CM);
        sleep(5);
        ACTII((int)l - 10);
    }

    void ACTV() {
        switch (pos) {
            case POS1:
                Bot.strafeDrive(40, .5, this);
                break;
            case POS2:
                Bot.strafeDrive(100, .5, this);
                break;
            case POS3:
                Bot.strafeDrive(160, .3, this);
        }
    }
}
