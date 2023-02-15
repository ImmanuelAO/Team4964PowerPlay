package org.firstinspires.ftc.teamcode.Robot;

public class Variables  {
    public int Lvl_Ground = -250,
            Lvl_Short  = -1500,
<<<<<<< Updated upstream
            Lvl_Mid    = -2300,
=======
            Lvl_Mid    = -2250,
>>>>>>> Stashed changes
            Lvl_Tall   = -2800;
    public boolean slo,
            btnlock = false;
    public int claw_zero = 23, //35,
            claw_cone = -28;
    static final double     COUNTS_PER_MOTOR_REV    = 19;
    static final double     DRIVE_GEAR_REDUCTION    = 40;
    static final double     WHEEL_DIAMETER_INCHES   = 4.125;
    static final double     EMPIRICAL_MULTIPLIER    = (30.0 / 17);
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * EMPIRICAL_MULTIPLIER)
            / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double conversion = 16.7;
}