package org.firstinspires.ftc.teamcode.auto.machines;

import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class StateMachineBuilder<E extends Enum<E>> {
    private E state;

    private final List<E> allStates = new ArrayList<>();

    private final Map<E, Runnable> enterMap = new HashMap<>();
    private final Map<E, Runnable> loopMap = new HashMap<>();
    private final Map<E, Runnable> exitMap = new HashMap<>();
    private final Map<E, Supplier<E>> exitToMap = new HashMap<>();

    private final Map<E, Long> msMap = new HashMap<>();
    private final Map<E, Supplier<Boolean>> supplierMap = new HashMap<>();

    public StateMachineBuilder<E> state(E state) {
        if (allStates.contains(state)) throw new IllegalArgumentException();
        allStates.add(this.state = state);
        return this;
    }

    public StateMachineBuilder<E> time(long ms) {
        if (state == null || supplierMap.containsKey(state)) throw new IllegalStateException();
        this.msMap.put(state, ms);
        return this;
    }

    public StateMachineBuilder<E> sample(Supplier<Boolean> supplier) {
        if (state == null || msMap.containsKey(state)) throw new IllegalStateException();
        this.supplierMap.put(state, supplier);
        return this;
    }

    public StateMachineBuilder<E> instant() {
        if (state == null || msMap.containsKey(state)) throw new IllegalStateException();
        this.supplierMap.put(state, () -> true);
        return this;
    }

    public StateMachineBuilder<E> onEnter(Runnable runnable) {
        if (state == null) throw new IllegalStateException();
        this.enterMap.put(state, runnable);
        return this;
    }

    public StateMachineBuilder<E> onLoop(Runnable runnable) {
        if (state == null) throw new IllegalStateException();
        this.loopMap.put(state, runnable);
        return this;
    }

    public StateMachineBuilder<E> onExit(Runnable runnable) {
        if (state == null) throw new IllegalStateException();
        this.exitMap.put(state, runnable);
        return this;
    }

    public StateMachineBuilder<E> exitTo(Supplier<E> supplier) {
        if (state == null) throw new IllegalStateException();
        this.exitToMap.put(state, supplier);
        return this;
    }

    public StateMachine<E> build() {
        List<State<E>> list = new ArrayList<>();

        for (E state : allStates) {
            if (msMap.containsKey(state)) list.add(
                    new TimedState<>(
                            state, msMap.get(state),
                            enterMap.getOrDefault(state, () -> {}),
                            loopMap.getOrDefault(state, () -> {}),
                            exitMap.getOrDefault(state, () -> {}),
                            exitToMap.get(state)
                    )
            );

            else if (supplierMap.containsKey(state)) list.add(
                    new SampleState<>(
                            state, supplierMap.get(state),
                            enterMap.getOrDefault(state, () -> {}),
                            loopMap.getOrDefault(state, () -> {}),
                            exitMap.getOrDefault(state, () -> {}),
                            exitToMap.get(state)
                    )
            );

            else throw new IllegalStateException();
        }

        return new StateMachine<>(list);
    }
}