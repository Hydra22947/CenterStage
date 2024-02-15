package org.firstinspires.ftc.teamcode.auto.Actions;

import org.firstinspires.ftc.teamcode.util.Stopwatch;

public class ActionHelper {
    public static boolean activateSystem(Stopwatch timer, Runnable systemFunction, long delay, Object... parameters) {
        if (timer.hasTimePassed(delay)) {
            systemFunction.run();
            timer.reset();
            return false; // Activation successful
        } else {
            return true; // Activation failed
        }
    }
}
