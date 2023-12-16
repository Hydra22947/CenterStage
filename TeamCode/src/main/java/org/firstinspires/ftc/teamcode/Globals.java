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

    public static double START_ELEVATOR = 2;
    public static double ELEVATOR_INCREMENT = 70;
    public static double ERROR = 2;
    public static double WAIT_DELAY_TILL_OUTTAKE = 150;
    public static double WAIT_DELAY_TILL_CLOSE = 250;
}
