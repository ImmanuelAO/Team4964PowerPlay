package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="teleOP(offseason)")
public class offseasonTeleOP extends OpMode {


    public static DcMotor bLWheel  = null;
    public static DcMotor bRWheel = null;

    @Override
    public void init() {
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialization In Progress");
        telemetry.update();

        //Hardware map
        bLWheel   = hardwareMap.get(DcMotor.class, "BackL");
        bRWheel  = hardwareMap.get(DcMotor.class, "BackR");

        bLWheel.setDirection(DcMotor.Direction.FORWARD);
        bRWheel.setDirection(DcMotor.Direction.REVERSE);

        bLWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bLWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bLWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bLWheel.setPower(0);
        bRWheel.setPower(0);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void loop() {

        double stopBuffer = 0; //Not currently Implemented

        //Drive Train Code
        double forward = Math.pow(gamepad1.left_stick_y, 3);
        double right = Math.pow(gamepad1.left_stick_x, 3);
        double turn = Math.pow(gamepad1.right_stick_x,3);

        double leftBackPower = forward - right + turn;
        double rightBackPower = forward + right - turn;
        double[] powers = {leftBackPower, rightBackPower};

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
            leftBackPower /= greatest;
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
            leftBackPower = 0;
            rightBackPower = 0;
        }

        bLWheel.setPower(leftBackPower);
        bRWheel.setPower(rightBackPower);
        //Drive Train Code

        telemetry.addLine("Back left encoder counts: " + bLWheel.getCurrentPosition());
        telemetry.addLine("Back right encoder counts: " + bRWheel.getCurrentPosition());
    }
}