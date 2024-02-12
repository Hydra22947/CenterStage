package org.firstinspires.ftc.teamcode.auto.rr1.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.Stopwatch;

public class DepositActions {


    public enum Cycles {
        PRELOAD,
        FIRST_CYCLE,
        SECOND_CYCLE
    }

    ;
    private Elevator elevator;
    private Intake intake;
    private Outtake outtake;
    private Claw claw;
    private Stopwatch timer;

    public Cycles currentCycle;

    public DepositActions(Elevator elevator, Intake intake, Claw claw, Outtake outtake) {
        this.elevator = elevator;
        this.intake = intake;
        this.claw = claw;
        this.outtake = outtake;

        this.timer = new Stopwatch();
        this.currentCycle = Cycles.PRELOAD;
    }


    private void moveElevatorByTraj() {
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


    //This function will prepare the intake and outtake  for deposit
    private void resetIntakeOuttake() {
        intake.move(Intake.Angle.MID);
        outtake.setAngle(Outtake.Angle.OUTTAKE);
    }


    private void retractElevator() {
        outtake.setAngle(Outtake.Angle.INTAKE);
        elevator.move(0);
    }

    //This function will get the function, its parameters and the delay and execute
    //this function with the delay.
    private boolean activateSystem(Runnable systemFunction, long delay, Object... parameters) {
        if (timer.hasTimePassed(delay)) {
            systemFunction.run();
            timer.reset();
            return false; // Activation successful
        } else {
            return true; // Activation failed
        }
    }


    public class ReadyForDeposit implements Action {

        public ReadyForDeposit() {
            timer.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return activateSystem(() -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH), 0) ||
                    activateSystem(() -> resetIntakeOuttake(), 200);
        }
    }

    public class PlacePixel implements Action {

        public PlacePixel(Cycles cycle) {
            timer.reset();
            currentCycle = cycle;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return activateSystem(() -> moveElevatorByTraj(), 100) ||
                    activateSystem(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH), 1000);
        }
    }

    public class RetractDeposit implements Action {

        public RetractDeposit() {
            timer.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return activateSystem(() -> retractElevator(), 800);
        }
    }
    public Action readyForDeposit()
    {
        return new ReadyForDeposit();
    }

    public Action placePixel(Cycles currentCycle)
    {
        return new PlacePixel(currentCycle);
    }

    public Action retractDeposit()
    {
        return new RetractDeposit();
    }
}
