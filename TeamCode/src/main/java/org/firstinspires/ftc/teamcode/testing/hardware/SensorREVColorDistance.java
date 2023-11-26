package org.firstinspires.ftc.teamcode.testing.hardware;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import org.firstinspires.ftc.teamcode.util.values.Globals;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Sensor: REVColorDistance", group = "tests")
public class SensorREVColorDistance extends LinearOpMode {

    RevColorSensorV3 rightSensorColor;
    RevColorSensorV3 leftSensorColor;
    // hsvValues arrays to hold the hue, saturation, and value information.
    float rightHsvValues[] = {0F, 0F, 0F};
    float leftHsvValues[] = {0F, 0F, 0F};
    @Override
    public void runOpMode() {

        // get references to the color sensors.
        rightSensorColor = hardwareMap.get(RevColorSensorV3.class, "cR");
        leftSensorColor = hardwareMap.get(RevColorSensorV3.class, "cL");



        // wait for the start button to be pressed.
        waitForStart();

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            Color.RGBToHSV((int) (rightSensorColor.red() * 255),
                    (int) (rightSensorColor.green() * 255),
                    (int) (rightSensorColor.blue() * 255),
                    rightHsvValues);

            Color.RGBToHSV((int) (leftSensorColor.red() * 255),
                    (int) (leftSensorColor.green() * 255),
                    (int) (leftSensorColor.blue() * 255),
                    leftHsvValues);

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
    }

    public boolean checkIfPixelIn(float[] hsvValues)
    {
        boolean hueCheck = ((hsvValues[0] + 5) == Globals.BASIC_HUE) || ((hsvValues[0] - 5) == Globals.BASIC_HUE);
        boolean satCheck = ((hsvValues[1] + 5) == Globals.BASIC_HUE) || ((hsvValues[1] - 5) == Globals.BASIC_HUE);
        boolean valCheck = ((hsvValues[2] + 5) == Globals.BASIC_HUE) || ((hsvValues[2] - 5) == Globals.BASIC_HUE);

        return hueCheck &&  satCheck && valCheck;
    }
}
