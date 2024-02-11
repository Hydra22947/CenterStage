package org.firstinspires.ftc.teamcode.auto.rr1.Actions;

import static org.firstinspires.ftc.teamcode.auto.rr1.Actions.DepositeAction.DepositeState.RESET_INTAKE_OUTTAKE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.Stopwatch;

public class DepositeAction implements Action {
    public enum DepositeState {
        OPEN_ELEVATOR,
        RESET_INTAKE_OUTTAKE,
        PLACE_PIXEL,
        RETRACT,
        IDLE
    }

    ;
    DepositeState depositeState = DepositeState.IDLE;
    Stopwatch timer;
    Intake intake;
    Outtake outtake;
    Elevator elevator;
    Claw claw;

    enum Cycles {
        PRELOAD,
        FIRST_CYCLE,
        SECOND_CYCLE
    }

    ;

    Cycles currentCycle = Cycles.PRELOAD;
    DepositeState depositState = DepositeState.IDLE;

    //This function will prepare the intake and outtake  for deposit
    private void resetIntakeOuttake() {
        intake.move(Intake.Angle.MID);
        outtake.setAngle(Outtake.Angle.OUTTAKE);
    }

    void moveElevatorByTraj() {
        switch (currentCycle) {
            case PRELOAD:
                elevator.move(1050);
                break;
            case FIRST_CYCLE:
                elevator.move(1500);
                break;
            case SECOND_CYCLE:
                elevator.move(1550);
                break;
        }
    }

    public void retractElevator() {
        outtake.setAngle(Outtake.Angle.INTAKE);
        elevator.move(0);
    }

    //This function will get the function, its parameters and the delay and execute
    //this function with the delay.
    private boolean activateSystem(Runnable systemFunction, long delay, Object... parameters) {
        if (timer.hasTimePassed(delay)) {
            systemFunction.run();
            timer.reset();
            return true; // Activation successful
        } else {
            return false; // Activation failed
        }
    }

    private void depositFSM() {
        timer = new Stopwatch();

        switch (depositeState) {
            case RESET_INTAKE_OUTTAKE:
                timer.reset();
                if (!activateSystem(() -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH), 0))
                    return;
                if (!activateSystem(() -> resetIntakeOuttake(), 200)) return;

                depositeState = DepositeState.RESET_INTAKE_OUTTAKE;
                break;
            case OPEN_ELEVATOR:
                if (!activateSystem(() -> moveElevatorByTraj(), 100)) return;
                depositeState = DepositeState.PLACE_PIXEL;
                break;
            case PLACE_PIXEL:
                if (!activateSystem(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH), 1000))
                    return;
                depositeState = DepositeState.RETRACT;
                break;
            case RETRACT:
                if (!activateSystem(() -> retractElevator(), 800)) return;
                depositeState = DepositeState.IDLE;
                break;
            case IDLE:
                break;
        }

    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        return false;
    }
}
