package org.firstinspires.ftc.teamcode.auto.machines;

import java.util.function.Supplier;

public abstract class State<E extends Enum<E>> {
    protected final E value;
    protected final Runnable enterAction,
                             loopAction,
                             exitAction;
    private final Supplier<E> exitToSupplier;

    protected State(E value, Runnable enterAction,
                    Runnable loopAction, Runnable exitAction,
                    Supplier<E> exitToSupplier) {
        this.value = value;
        this.enterAction = enterAction;
        this.loopAction = loopAction;
        this.exitAction = exitAction;
        this.exitToSupplier = exitToSupplier;
    }

    public abstract boolean shouldTransition();

    public void onEnterState() {}

    public E getValue() {
        return value;
    }

    public Runnable getEnterAction() {
        return enterAction;
    }

    public Runnable getLoopAction() {
        return loopAction;
    }

    public Runnable getExitAction() {
        return exitAction;
    }

    public Supplier<E> getExitToSupplier() {
        return exitToSupplier;
    }
}
