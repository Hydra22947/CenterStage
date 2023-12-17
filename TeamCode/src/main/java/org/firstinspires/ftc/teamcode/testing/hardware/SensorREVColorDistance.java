package org.firstinspires.ftc.teamcode.testing.hardware;

import static org.firstinspires.ftc.teamcode.subsystems.Intake.seeFarFrom;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Sensor: REVColorDistance", group = "tests")
@Disabled
public class SensorREVColorDistance extends LinearOpMode {

    RevColorSensorV3 rightSensorColor;
    RevColorSensorV3 leftSensorColor;
    public static double toleranceH = 5; // Range of +-5
    public static double toleranceS = 5; // Range of +-5
    public static double toleranceV = 5; // Range of +-5

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
            telemetry.addData("Right Sensor - DISTANCE", rightSensorColor.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Sensor - DISTANCE", leftSensorColor.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Sensor - V", leftHsvValues[2]);
            telemetry.addData("Right Detected", checkIfPixelIn(rightSensorColor));
            telemetry.addData("Left Detected", checkIfPixelIn(leftSensorColor));

            telemetry.update();
        }
    }

//    public boolean checkIfPixelIn(float[] hsvValues)
//    {
//
//
//        boolean hueCheck = hsvValues[0] >= Globals.BASIC_HUE - toleranceH && hsvValues[0] <= Globals.BASIC_HUE + toleranceH;
//        boolean satCheck = hsvValues[1] >= Globals.BASIC_SAT - toleranceS && hsvValues[0] <= Globals.BASIC_SAT + toleranceS;
//        boolean valCheck = hsvValues[2] >= Globals.BASIC_VAL - toleranceV && hsvValues[0] <= Globals.BASIC_VAL + toleranceV;
//
//        return !(hueCheck &&  satCheck && valCheck);
//    }
    public boolean checkIfPixelIn(RevColorSensorV3 sensor)
    {
        return -seeFarFrom <= sensor.getDistance(DistanceUnit.CM) && sensor.getDistance(DistanceUnit.CM) <= seeFarFrom;
    }
}
