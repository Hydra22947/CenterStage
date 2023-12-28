package org.firstinspires.ftc.teamcode.util;

public class ActionTimer {
    private final long ms;
    private final Runnable whenTimer;
    private final Stopwatch stopwatch;

    public ActionTimer(long ms, Runnable whenTimer) {
        this.ms = ms;
        this.whenTimer = whenTimer;

        this.stopwatch = new Stopwatch();
    }

    public boolean update() {
        if (stopwatch.hasTimePassed(ms)) {
            whenTimer.run();

            return true;
        }

        return false;
    }
}
