package org.firstinspires.ftc.teamcode.testing.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Concept: Gamepad Rumble", group ="Concept")
public class ConceptGamepadRumble extends LinearOpMode
{
    boolean lastA = false;                      // Use to track the prior button state.
    boolean lastLB = false;                     // Use to track the prior button state.
    boolean highLevel = false;                  // used to prevent multiple level-based rumbles.
    boolean secondHalf = false;                 // Use to prevent multiple half-time warning rumbles.

    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
    ElapsedTime runtime = new ElapsedTime();    // Use to determine when end game is starting.

    final double HALF_TIME = 60.0;              // Wait this many seconds before rumble-alert for half-time.
    final double TRIGGER_THRESHOLD  = 0.75;     // Squeeze more than 3/4 to get rumble.

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

        telemetry.addData(">", "Press Start");
        telemetry.update();

        waitForStart();
        runtime.reset();    // Start game timer.

        // Loop while monitoring buttons for rumble triggers
        while (opModeIsActive())
        {
            // Read and save the current gamepad button states.
            boolean currentA = gamepad1.a ;
            boolean currentLB = gamepad1.left_bumper ;

            // Display the current Rumble status.  Just for interest.
            telemetry.addData(">", "Are we RUMBLING? %s\n", gamepad1.isRumbling() ? "YES" : "no" );

            // ----------------------------------------------------------------------------------------
            // Example 1. b) Watch the runtime timer, and run the custom rumble when we hit half-time.
            //               Make sure we only signal once by setting "secondHalf" flag to prevent further rumbles.
            // ----------------------------------------------------------------------------------------
            if ((runtime.seconds() > HALF_TIME) && !secondHalf)  {
                gamepad1.runRumbleEffect(customRumbleEffect);
                secondHalf =true;
            }

            // Display the time remaining while we are still counting down.
            if (!secondHalf) {
                telemetry.addData(">", "Halftime Alert Countdown: %3.0f Sec \n", (HALF_TIME - runtime.seconds()) );
            }


            // ----------------------------------------------------------------------------------------
            // Example 2. If Left Bumper is being pressed, power the rumble motors based on the two trigger depressions.
            // This is useful to see how the rumble feels at various power levels.
            // ----------------------------------------------------------------------------------------
            if (currentLB) {
                // Left Bumper is being pressed, so send left and right "trigger" values to left and right rumble motors.
                gamepad1.rumble(gamepad1.left_trigger, gamepad1.right_trigger, Gamepad.RUMBLE_DURATION_CONTINUOUS);

                // Show what is being sent to rumbles
                telemetry.addData(">", "Squeeze triggers to control rumbles");
                telemetry.addData("> : Rumble", "Left: %.0f%%   Right: %.0f%%", gamepad1.left_trigger * 100, gamepad1.right_trigger * 100);
            } else {
                // Make sure rumble is turned off when Left Bumper is released (only one time each press)
                if (lastLB) {
                    gamepad1.stopRumble();
                }

                //  Prompt for manual rumble action
                telemetry.addData(">", "Hold Left-Bumper to test Manual Rumble");
                telemetry.addData(">", "Press A (Cross) for three blips");
                telemetry.addData(">", "Squeeze right trigger slowly for 1 blip");
            }
            lastLB = currentLB; // remember the current button state for next time around the loop


            // ----------------------------------------------------------------------------------------
            // Example 3. Blip 3 times at the moment that A (Cross) is pressed. (look for pressed transition)
            // BUT !!!  Skip it altogether if the Gamepad is already rumbling.
            // ----------------------------------------------------------------------------------------
            if (currentA && !lastA) {
                if (!gamepad1.isRumbling())  // Check for possible overlap of rumbles.
                    gamepad1.rumbleBlips(3);
            }
            lastA = currentA; // remember the current button state for next time around the loop


            // ----------------------------------------------------------------------------------------
            // Example 4. Rumble once when gamepad right trigger goes above the THRESHOLD.
            // ----------------------------------------------------------------------------------------
            if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                if (!highLevel) {
                    gamepad1.rumble(0.9, 0, 200);  // 200 mSec burst on left motor.
                    highLevel = true;  // Hold off any more triggers
                }
            } else {
                highLevel = false;  // We can trigger again now.
            }

            // Send the telemetry data to the Driver Station, and then pause to pace the program.
            telemetry.update();
            sleep(10);
        }
    }
}