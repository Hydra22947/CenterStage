package org.firstinspires.ftc.teamcode.auto.machines;

import java.util.function.Supplier;

public class SampleState<E extends Enum<E>> extends State<E> {
    private final Supplier<Boolean> supplier;

    public SampleState(E value, Supplier<Boolean> supplier, Runnable enterAction, Runnable loopAction, Runnable exitAction, Supplier<E> exitToSupplier) {
        super(value, enterAction, loopAction, exitAction, exitToSupplier);

        this.supplier = supplier;
    }

    @Override
    public boolean shouldTransition() {
        return supplier.get();
    }
}
