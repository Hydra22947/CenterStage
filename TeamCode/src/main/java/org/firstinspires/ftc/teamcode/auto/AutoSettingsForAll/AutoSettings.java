package org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.auto.BasicAutos.AutoBlueLeft;

import java.io.File;

public class AutoSettings {

    public enum PropLocation {
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

