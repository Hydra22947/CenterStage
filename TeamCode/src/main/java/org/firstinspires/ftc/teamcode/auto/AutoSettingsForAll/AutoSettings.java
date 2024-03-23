package org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.auto.BasicAutos.AutoBlueLeft;

import java.io.File;

public class AutoSettings {

    public enum PropLocation
    {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static PropLocation cycleVision(PropLocation propLocation)
    {
        switch (propLocation)
        {
            case LEFT:
                return PropLocation.MIDDLE;
            case MIDDLE:
                return PropLocation.RIGHT;
            case RIGHT:
                return PropLocation.LEFT;
        }

        return propLocation;
    }

    public enum AllianceColor
    {
        BLUE,
        RED
    }

    public enum AllianceSide
    {
        FAR,
        CLOSE
    }

    public static void writeToFile (double myNumber) {
        File myFileName = AppUtil.getInstance().getSettingsFile("AUTO_LAST_ANGLE.txt");

        ReadWriteFile.writeFile(myFileName, String.valueOf(myNumber));
    }

    public static double readFromFile () {
        File myFileName = AppUtil.getInstance().getSettingsFile("AUTO_LAST_ANGLE.txt");

        double myNumber = Double.parseDouble(ReadWriteFile.readFile(myFileName).trim());

        return myNumber;

    }
}
