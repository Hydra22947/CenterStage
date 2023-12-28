package org.firstinspires.ftc.teamcode.auto.machines;

import java.util.List;

public class StateMachine<E extends Enum<E>> {
    private final List<State<E>> stateList;
    private State<E> currentState = null;
    private int index = -1;
    private boolean done = false;

    StateMachine(List<State<E>> stateList) {
        this.stateList = stateList;
    }

    public void update() {
        if (currentState == null || currentState.shouldTransition()) {
            int nextIndex = index + 1;

            if (currentState != null) {
                currentState.getExitAction().run();

                if (currentState.getExitToSupplier()  != null) {
                    E target = currentState.getExitToSupplier().get();
                    if (target != null) {
                        int i = 0;

                        for (State<E> state : stateList) {
                            if (state.getValue() == target) {
                                nextIndex = i;
                                break;
                            }

                            i++;
                        }
                    }
                }
            }

            if (nextIndex >= stateList.size()) {
                done = true;

                return;
            }

            (currentState = stateList.get(nextIndex)).getEnterAction().run();
            currentState.onEnterState();

            index = nextIndex;
        } else currentState.getLoopAction().run();
    }

    public boolean setState(E state) {
        if (!stateList.contains(state)) return false;

        State<E> newState = currentState;

        for (State<E> s : stateList) {
            if (s.value == state) {
                newState = s;

                break;
            }
        }

        if (newState == currentState) return false;

        currentState.getExitAction().run();

        newState.getEnterAction().run();
        newState.onEnterState();

        currentState = newState;
        index = stateList.indexOf(currentState);

        return true;
    }

    public E getState() {
        return currentState.value;
    }

    public boolean isDone() {
        return done;
    }

    public boolean isNotDone() {
        return !isDone();
    }
}
