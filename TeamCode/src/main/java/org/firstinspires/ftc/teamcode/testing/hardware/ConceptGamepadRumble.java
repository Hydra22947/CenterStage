package org.firstinspires.ftc.teamcode.testing.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Concept: Gamepad Rumble", group ="Concept")
@Disabled
public class ConceptGamepadRumble extends LinearOpMode
{
    boolean secondHalf = false;                 // Use to prevent multiple half-time warning rumbles.

    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
    ElapsedTime runtime = new ElapsedTime();    // Use to determine when end game is starting.

    final double HALF_TIME = 60.0;              // Wait this many seconds before rumble-alert for half-time.

    @Override
    public void runOpMode()
    {
        // Example 1. a)   start by creating a three-pulse rumble sequence: right, LEFT, LEFT
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();


        waitForStart();
        runtime.reset();    // Start game timer.

        // Loop while monitoring buttons for rumble triggers
        while (opModeIsActive())
        {

            // ----------------------------------------------------------------------------------------
            // Example 1. b) Watch the runtime timer, and run the custom rumble when we hit half-time.
            //               Make sure we only signal once by setting "secondHalf" flag to prevent further rumbles.
            // ----------------------------------------------------------------------------------------
            if ((runtime.seconds() > HALF_TIME) && !secondHalf)  {
                gamepad1.runRumbleEffect(customRumbleEffect);
                secondHalf =true;
            }


            // Send the telemetry data to the Driver Station, and then pause to pace the program.
            telemetry.update();
            sleep(10);
        }
    }
}