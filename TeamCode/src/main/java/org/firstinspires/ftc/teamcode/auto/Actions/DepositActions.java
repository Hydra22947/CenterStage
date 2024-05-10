package org.firstinspires.ftc.teamcode.auto.Actions;

import androidx.annotation.NonNull;
import static org.firstinspires.ftc.teamcode.auto.Actions.ActionHelper.activateSystem;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.Stopwatch;

public class DepositActions {


    public enum Cycles {
        PRELOAD,
        FIRST_CYCLE,
        SECOND_CYCLE
    }

    ;
    private LiftSubsystem elevator;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    private Claw claw;

    private IntakeExtensionSubsystem extension;

    private boolean activated;


    public DepositActions(LiftSubsystem elevator, IntakeSubsystem intake, Claw claw, OuttakeSubsystem outtake, IntakeExtensionSubsystem extension) {
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
        intake.move(IntakeSubsystem.Angle.MID);
        outtake.setAngle(OuttakeSubsystem.Angle.OUTTAKE);
    }


    public void retractElevator() {
        outtake.setAngle(OuttakeSubsystem.Angle.INTAKE);
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
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH);
            moveElevatorByTraj(elevator);
            outtake.setAngle(OuttakeSubsystem.Angle.OUTTAKE);

            return false;
        }
    }


    public class MoveOuttake implements Action {
        Stopwatch readyForDepositTimer;
        OuttakeSubsystem.Angle angle;

        public MoveOuttake (OuttakeSubsystem.Angle angle) {
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
    public class PlaceIntermediatePixel implements Action {
        Stopwatch placePixelTimer;
        long delay = 0;

        public PlaceIntermediatePixel(Cycles current, long d) {
            placePixelTimer = new Stopwatch();
            placePixelTimer.reset();
            delay = d;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return !activateSystem(placePixelTimer, () -> claw.updateState(Claw.ClawState.INTERMEDIATE, ClawSide.BOTH), delay);
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
    public Action placeIntermediatePixel(Cycles currentCycle, long d) {
        return new PlaceIntermediatePixel(currentCycle, d);
    }
    public Action moveOuttake (OuttakeSubsystem.Angle thisAngle) { return new MoveOuttake(thisAngle); }

    public Action moveElevator (int thisTarget) { return new MoveElevator(thisTarget); }

}
