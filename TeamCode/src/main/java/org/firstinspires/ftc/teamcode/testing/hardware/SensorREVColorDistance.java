package org.firstinspires.ftc.teamcode.testing.hardware;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Sensor: REVColorDistance", group = "tests")
public class SensorREVColorDistance extends LinearOpMode {

    RevColorSensorV3 rightSensorColor;
    RevColorSensorV3 leftSensorColor;

    @Override
    public void runOpMode() {

        // get references to the color sensors.
        rightSensorColor = hardwareMap.get(RevColorSensorV3.class, "dsR");
        leftSensorColor = hardwareMap.get(RevColorSensorV3.class, "dsL");

        // hsvValues arrays to hold the hue, saturation, and value information.
        float rightHsvValues[] = {0F, 0F, 0F};
        float leftHsvValues[] = {0F, 0F, 0F};

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

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
            telemetry.addData("Right argb  ", rightSensorColor.argb());
            telemetry.addData("Right Sensor - Hue", rightHsvValues[0]);

            telemetry.addData("Left argb  ", leftSensorColor.argb());
            telemetry.addData("Left Sensor - Hue", leftHsvValues[0]);

            // change the background color to match the hue detected by the right RGB sensor.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, new float[]{leftHsvValues[0], 1.0f, 1.0f}));
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, new float[]{rightHsvValues[0], 1.0f, 1.0f}));
                }
            });

            telemetry.update();
        }

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}
