package org.firstinspires.ftc.teamcode.Robot;

public class Variables  {
    public int Lvl_Ground = -100,
            Lvl_Short  = -1000,
            Lvl_Mid    = -2000,
            Lvl_Tall   = -3000;
    public boolean slo,
            btnlock = false;
    public int claw_zero = 25, //35,
            claw_cone = -28;
    static final double     COUNTS_PER_MOTOR_REV    = 19; //gear ratio = 1.00
    static final double     DRIVE_GEAR_REDUCTION    = 40;
    static final double     WHEEL_DIAMETER_INCHES   = 4.125;
    static final double     EMPIRICAL_MULTIPLIER    = (30.0 / 17);
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * EMPIRICAL_MULTIPLIER)
            / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double conversion = 16.7; //change to 17.4592 for more accurate measures
    //(12.125in circumference / 537.7 encoder counts per rev) = 44.34639in encoder per in
    //(44.34639 / 2.54) = 17.4592 encoders per cm
}