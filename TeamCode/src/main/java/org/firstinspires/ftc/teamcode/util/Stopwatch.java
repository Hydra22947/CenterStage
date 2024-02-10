package org.firstinspires.ftc.teamcode.util;

public class Stopwatch {
    private long startTime = System.currentTimeMillis();

    public long getRuntime() {
        return System.currentTimeMillis() - startTime;
    }

    public boolean hasTimePassed(long ms) {
        return getRuntime() >= ms;
    }

    public void reset() {
        startTime = System.currentTimeMillis();
    }
}
