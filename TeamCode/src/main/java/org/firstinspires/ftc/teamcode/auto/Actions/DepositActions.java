package org.firstinspires.ftc.teamcode.auto.Actions;

import androidx.annotation.NonNull;
import static org.firstinspires.ftc.teamcode.auto.Actions.ActionHelper.activateSystem;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
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

    private IntakeExtension extension;

    private boolean activated;


    public DepositActions(Elevator elevator, Intake intake, Claw claw, Outtake outtake, IntakeExtension extension) {
        this.elevator = elevator;
        this.intake = intake;
        this.extension = extension;
        this.claw = claw;
        this.outtake = outtake;
    }


    private void moveElevatorByTraj(int elevatorTarget) {
        elevator.setTarget(elevatorTarget);
        elevator.setPidControl();
    }

    //This function will prepare the intake and outtake  for deposit
    private void resetIntakeOuttake() {
        intake.move(Intake.Angle.MID);
        outtake.setAngle(Outtake.Angle.OUTTAKE);
    }


    private void retractElevator() {
        outtake.setAngle(Outtake.Angle.INTAKE);
        elevator.setTarget(0);
        elevator.setPidControl();
    }



    public class ReadyForDeposit implements Action {
        Stopwatch readyForDepositTimer;
        int elevator;

        public ReadyForDeposit(int elevator) {
            this.elevator = elevator;
            readyForDepositTimer = new Stopwatch();
            readyForDepositTimer.reset();
            intake.move(Intake.Angle.MID);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH);
            moveElevatorByTraj(elevator);
            outtake.setAngle(Outtake.Angle.OUTTAKE);

            return false;
        }
    }


    public class MoveOuttake implements Action {
        Stopwatch readyForDepositTimer;
        Outtake.Angle angle;

        public MoveOuttake (Outtake.Angle angle) {
            this.angle = angle;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outtake.setAngle(angle);
            return false;
        }
    }


    public class PlacePixel implements Action {
        Stopwatch placePixelTimer;
        long delay = 0;

        public PlacePixel(Cycles current, long d) {
            placePixelTimer = new Stopwatch();
            placePixelTimer.reset();
            delay = d;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return !activateSystem(placePixelTimer, () -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH), delay);
        }
    }


    public class RetractDeposit implements Action {
        Stopwatch retractDepositTimer;

        public RetractDeposit() {
            retractDepositTimer = new Stopwatch();
            retractDepositTimer.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return !activateSystem(retractDepositTimer, () -> retractElevator(), 800);


        }
    }


    public class MoveElevator implements Action {
        Stopwatch retractDepositTimer;
        int target;
        public MoveElevator (int target)
        {
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            moveElevatorByTraj(target);
            return false;

        }
    }
    public Action retractDeposit() {
        return new RetractDeposit();
    }

    public Action readyForDeposit(int elevator) {
        return new ReadyForDeposit(elevator);
    }

    public Action placePixel(Cycles currentCycle, long d) {
        return new PlacePixel(currentCycle, d);
    }

    public Action moveOuttake (Outtake.Angle thisAngle) { return new MoveOuttake(thisAngle); }

    public Action moveElevator (int thisTarget) { return new MoveElevator(thisTarget); }

}
