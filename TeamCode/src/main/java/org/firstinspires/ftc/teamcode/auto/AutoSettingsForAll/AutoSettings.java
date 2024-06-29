package org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.units.qual.C;

@Config
public class AutoSettings {

    public static enum PropLocation {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static PropLocation cycleVision(PropLocation propLocation) {
        switch (propLocation) {
            case LEFT:
                return PropLocation.MIDDLE;
            case MIDDLE:
                return PropLocation.RIGHT;
            case RIGHT:
                return PropLocation.LEFT;
        }

        return propLocation;
    }

    public enum AllianceColor {
        BLUE,
        RED
    }

    public enum AllianceSide {
        FAR,
        CLOSE
    }
}

