package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.Side;

@Config
public class Globals {

    /**
     * Match constants.
     */
    public static Side SIDE = Side.LEFT;
    public static boolean IS_AUTO = false;
    public static boolean IS_USING_IMU = true;
    public static boolean USING_DASHBOARD = false;

    public static double MAX_POWER = 0.8;
    public static double START_ELEVATOR = 2;
    public static double ELEVATOR_INCREMENT = .2;
    public static double BASIC_HUE = 72, BASIC_SAT = 0.61, BASIC_VAL = 31;
    public static double SCALE_FACTOR = 255;
    public static double WAIT_DELAY_TILL_OUTTAKE = 125;
    public static double WAIT_DELAY_TILL_CLOSE = 500;
    public static double INCREMENT = 105;
    public static double delayClaw = 1;
    /*Pixel hsv values in case we need

    public static double YELLOW_HUE = 48, YELLOW_SAT = 0.77, YELLOW_VALUE = 13;
    public static double WHITE_HUE = 60, WHITE_SAT = 1, WHITE_VALUE = 3;
    public static double PURPLE_HUE = 60, PURPLE_SAT = 1, PURPLE_VALUE = 1;
    public static double GREEN_HUE = 120, GREEN_SAT = 0.5, GREEN_VALUE = 2;
    */
}
