package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.values.Globals;

public class ColorSensors {

    private final RobotHardware robot;

    // hsvValues arrays to hold the hue, saturation, and value information.
    private float rightHsvValues[] = {0F, 0F, 0F};
    private float leftHsvValues[] = {0F, 0F, 0F};
    public ColorSensors()
    {
        robot = RobotHardware.getInstance();
        this.insertHSVValues();
    }

    public void insertHSVValues()
    {
        Color.RGBToHSV((int) (this.robot.colorRight.red() * Globals.SCALE_FACTOR),
                (int) ((this.robot.colorRight.green() * Globals.SCALE_FACTOR)),
                (int) ((this.robot.colorRight.blue() * Globals.SCALE_FACTOR)),
                rightHsvValues);

        Color.RGBToHSV((int) (this.robot.colorLeft.red() * Globals.SCALE_FACTOR),
                (int) (this.robot.colorLeft.green() * Globals.SCALE_FACTOR),
                (int) (this.robot.colorLeft.blue() * Globals.SCALE_FACTOR),
                leftHsvValues);
    }
    public boolean checkIfPixelIn(float[] hsvValues)
    {
        boolean hueCheck = ((hsvValues[0] + 5) == Globals.BASIC_HUE) || ((hsvValues[0] - 5) == Globals.BASIC_HUE);
        boolean satCheck = ((hsvValues[1] + 5) == Globals.BASIC_HUE) || ((hsvValues[1] - 5) == Globals.BASIC_HUE);
        boolean valCheck = ((hsvValues[2] + 5) == Globals.BASIC_HUE) || ((hsvValues[2] - 5) == Globals.BASIC_HUE);

        return hueCheck &&  satCheck && valCheck;
    }
    public void getTelemetry()
    {

        // send the info back to driver station using telemetry function.
        telemetry.addData("Right Sensor - H", rightHsvValues[0]);
        telemetry.addData("Right Sensor - S", rightHsvValues[1]);
        telemetry.addData("Right Sensor - V", rightHsvValues[2]);
        telemetry.addData("Left Sensor - H", leftHsvValues[0]);
        telemetry.addData("Left Sensor - S", leftHsvValues[1]);
        telemetry.addData("Left Sensor - V", leftHsvValues[2]);
        telemetry.addData("Right Detected - V", checkIfPixelIn(rightHsvValues));
        telemetry.addData("Left Detected - V", checkIfPixelIn(leftHsvValues));

        telemetry.update();
    }

    public float[] getRightHsvValues()
    {
        return this.rightHsvValues;
    }

    public float[] getLeftHsvValues()
    {
        return this.leftHsvValues;
    }
}
