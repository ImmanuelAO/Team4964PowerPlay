package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Teleop")
    public class TylerTeleop extends OpMode {


        public DcMotor backleft;
        public DcMotor frontleft;
        public DcMotor backright;
        public DcMotor frontright;
        public DcMotor lift;
        public DcMotor leftgrabber;
        public DcMotor rightgrabber;


        public double speedMode = 1;
        public boolean xIsHeld = false;
        public boolean bIsHeld = false;
        public boolean dpadLeftIsHeld = false;
        public boolean dpadRightIsHeld = false;




        @Override
        public void init() {
            telemetry.clearAll();
            telemetry.addData("Status", "TeleOP Initialization In Progress");
            telemetry.update();

            //Hardware map
            frontleft   = hardwareMap.get(DcMotor.class, "FrontL");
            backleft   = hardwareMap.get(DcMotor.class, "BackL");
            frontright  = hardwareMap.get(DcMotor.class, "FrontR");
            backright  = hardwareMap.get(DcMotor.class, "BackR");
            lift      = hardwareMap.get(DcMotor.class, "lift"    );
            leftgrabber      = hardwareMap.get(DcMotor.class,   "claw"    );



            backleft.setDirection(DcMotor.Direction.FORWARD);
            frontleft.setDirection(DcMotor.Direction.FORWARD);
            backright.setDirection(DcMotor.Direction.FORWARD);
            frontright.setDirection(DcMotor.Direction.FORWARD);
            lift.setDirection(DcMotor.Direction.FORWARD);
            leftgrabber.setDirection(DcMotor.Direction.FORWARD);



            backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftgrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftgrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftgrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



            backleft.setPower(0);
            frontleft.setPower(0);
            backright.setPower(0);
            frontright.setPower(0);
            lift.setPower(0);
            leftgrabber.setPower(0);



            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");
            telemetry.update();

        }


        @Override
        public void loop() {

            //Slow Mode Code for a and b keys
            if (gamepad1.a == true) {
                speedMode = .4;
            } else if (gamepad1.b == true) {
                speedMode = 1;
            }
            //Slow Mode Code for a and b keys




            //Slow Mode Code for bumpers
            if (gamepad1.right_bumper == true && speedMode > .2) {
                speedMode -= .05;
            } else if (gamepad1.right_trigger >= .5 && speedMode < 2) {
                speedMode += .05;
            }
            //Slow Mode Code for bumpers


            double stopBuffer = 0; //Not currently Implemented



            //Drive Train Code
            double forward = speedMode * Math.pow(gamepad1.left_stick_y, 3);
            double right = -speedMode * Math.pow(gamepad1.left_stick_x, 3);
            double turn = -speedMode * Math.pow(gamepad1.right_stick_x,3);

            double leftFrontPower = forward + right + turn;
            double leftBackPower = forward - right + turn;
            double rightFrontPower = forward - right - turn;
            double rightBackPower = forward + right - turn;
            double[] powers = {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};

            boolean needToScale = false;
            for (double power : powers){
                if(Math.abs(power) > 1){
                    needToScale = true;
                    break;
                }
            }
            if (needToScale){
                double greatest = 0;
                for (double power : powers){
                    if (Math.abs(power) > greatest){
                        greatest = Math.abs(power);
                    }
                }
                leftFrontPower /= greatest;
                leftBackPower /= greatest;
                rightFrontPower /= greatest;
                rightBackPower /= greatest;
            }

            boolean stop = true;
            for (double power : powers){
                if (Math.abs(power) > stopBuffer){
                    stop = false;
                    break;
                }
            }
            if (stop){
                leftFrontPower = 0;
                leftBackPower = 0;
                rightFrontPower = 0;
                rightBackPower = 0;
            }

            frontleft.setPower(leftFrontPower);
            backleft.setPower(leftBackPower);
            frontright.setPower(rightFrontPower);
            backright.setPower(rightBackPower);

            telemetry.addLine("top left encoder counts: " + frontleft.getCurrentPosition());
            telemetry.addLine("bottom left encoder counts: " + backleft.getCurrentPosition());
            telemetry.addLine("top right encoder counts: " + frontright.getCurrentPosition());
            telemetry.addLine("bottom right encoder counts: " + backright.getCurrentPosition());
            //Drive Train Code


            //lift code
            if (gamepad2.left_stick_y >= 0.3) {
                lift.setPower(.5);
            } else if (gamepad2.left_stick_y <= -0.3) {
                lift.setPower(-.5);
            }else{
                lift.setPower(0);
            }


            //wormhole in
            if (gamepad2.b){
                leftgrabber.setPower(1);


            }else if (gamepad2.x){
                leftgrabber.setPower(-1);

            }else {
                leftgrabber.setPower(0);

            }

            telemetry.addData("claw", leftgrabber.getCurrentPosition()); // Jill's attempt to get claw limit positions. 


        }
    }

