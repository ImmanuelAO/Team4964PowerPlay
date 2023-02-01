package org.firstinspires.ftc.teamcode.Auto.Field;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Bot;
import org.firstinspires.ftc.teamcode.Robot.Variables;


@Autonomous(name= "Left Parking Auto")
public class ParkingAutoLeft extends LinearOpMode {

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

        if (ACTI()) {
            ACTII();
            ACTIII();
        }
        else {
            ACTIV();
            ACTV();
        }
    }

    boolean ACTI() {
        //close claw to hold cone
        Bot.Claw.setTargetPosition(var.claw_cone);
        sleep(1);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Bot.Claw.setPower(-1);
        sleep(550);
        //strafe to the right
        Bot.strafeDrive(75, .4, this);
        sleep(5);
        //drive forward
        Bot.driveStraight(125, .5, this);
        sleep(5);
        //begin tracking the distance with the sensor
        Bot.distance.getDistance(DistanceUnit.CM);
        int i = 80;
        //safety check for opposing robots in the way
        while (opModeIsActive() && Bot.distance.getDistance(DistanceUnit.CM) < 40 && i > 0) {
            sleep(50);
            i--;
        }
        //commands for when the opposing robot is still in the way
        if (i == 0) {
            Bot.SensorStrafeDrive(-50, 0.2, this);
            Bot.strafeDrive(-12, .3, this);
            return false;
        }
        //commands for when there is no robot blocking the path
        Bot.SensorStrafeDrive(-50, .2, this);
        //Bot.strafeDrive(-15, 0.3, this);
        return true;
    }
    void ACTII(){
        //raise lift to place cone
        Bot.Lift.setTargetPosition(var.Lvl_Tall);
        sleep(1);
        Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Lift.setPower(1);
        sleep(-var.Lvl_Tall);
        //drive up to the junction
        Bot.driveStraight(17, .3, this);
        //Bot.Lift.setTargetPosition(var.Lvl_Tall + 600);
        sleep(75);

        //Bot.strafeDrive(3,.5,this);

        //release cone onto the junction
        Bot.Claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bot.Claw.setTargetPosition(var.claw_zero);
        sleep(1);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Claw.setPower(1);
        sleep(200);
        //back away from the junction
        Bot.driveStraight(-15, .3, this);
        sleep(1);
        //close the claw
        Bot.Claw.setTargetPosition(var.claw_cone);
        sleep(1200);
        //lower the lift
        Bot.Lift.setTargetPosition(var.Lvl_Ground);
        //sleep(-var.Lvl_Tall);
        Bot.Lift.setPower(0);
    }

    void ACTIII(){
        switch (pos) {
            case POS1:
                Bot.strafeDrive(-98,.5,this);
                break;
            case POS2:
                Bot.strafeDrive(-40,.5,this);
                break;
            case POS3:
                Bot.strafeDrive(35,.5,this);
        }
    }

    void ACTIV(){
        //Bot.driveStraight(-200, .5, this);
        //activate distance sensor maybe?
        //Bot.strafeDrive(200, .5, this);
        //Bot.strafeDrive(-200, .5, this);
        //Bot.Lift.setTargetPosition(var.Lvl_Tall);
        //sleep(1);
        //move forward a little
//        Bot.Claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Bot.Claw.setTargetPosition(var.claw_zero);
//        sleep(1);
//        Bot.Claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Bot.Claw.setPower(1);
//        sleep(200);
        //lower lift and close claw
    }

    void ACTV(){ //change values before running or pushing code!!!!!!!!!!!!!
        switch (pos) {
            case POS1:
                Bot.strafeDrive(200, .9, this);
                break;
            case POS2:
                Bot.strafeDrive(200, .9, this);
                break;
            case POS3:
                Bot.strafeDrive(200, .9, this);
        }
    }
}