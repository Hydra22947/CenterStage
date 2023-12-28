package org.firstinspires.ftc.teamcode.auto.machines;


import org.firstinspires.ftc.teamcode.util.Stopwatch;

import java.util.function.Supplier;

public class TimedState<E extends Enum<E>> extends State<E> {
    private final Stopwatch stopwatch;
    private final long runningMS;

    public TimedState(E value, long runningMS, Runnable enterAction, Runnable loopAction, Runnable exitAction, Supplier<E> exitToSupplier) {
        super(value, enterAction, loopAction, exitAction, exitToSupplier);

        if (runningMS < 0) throw new IllegalArgumentException();

        stopwatch = new Stopwatch();

        this.runningMS = runningMS;
    }

    @Override
    public boolean shouldTransition() {
        return stopwatch.hasTimePassed(runningMS);
    }

    @Override
    public void onEnterState() {
        stopwatch.reset();
    }
}
