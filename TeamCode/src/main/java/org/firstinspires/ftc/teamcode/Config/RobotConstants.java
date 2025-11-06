package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static final double GearRatio = 1;

    public static double FLYWHEEL_Full = 1.0;
    public static double FLYWHEEL_Half = 0.8;
    public static double FLYWHEEL_OFF = 0;

    public static double INTAKE_Full = 0.8;
    public static double INTAKE_Half = 0.6;
    public static double INTAKE_OFF = 0;

    public static double Gate_Off = 0.0;
    public static double Gate_On = .27;

    public static double c_DriveSpeed = 0.9;
    public static double c_FL_WeightTuning = 1.0;
    public static double c_FR_WeightTuning = 1.0;
    public static double c_BL_WeightTuning = 1.0;
    public static double c_BR_WeightTuning = 1.0;

    public static int recoveryPause = 167; // ms
    public static int recoveryDelay = 1475; //ms

    public static final double initPower = 0.5;
}
