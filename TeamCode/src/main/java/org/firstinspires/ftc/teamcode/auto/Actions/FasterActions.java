package org.firstinspires.ftc.teamcode.auto.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

public class FasterActions {

    private Elevator elevator;
    private Intake intake;
    private Outtake outtake;
    private Claw claw;

    private IntakeExtension extension;


    DepositActions depositActions;
    ActionHelper actionHelper;
    PlacePurpleActions placePurpleActions;
    UpdateActions updateActions;

    public FasterActions(Elevator elevator, Intake intake, Claw claw, Outtake outtake, IntakeExtension extension) {

        this.elevator = elevator;
        this.intake = intake;
        this.extension = extension;
        this.claw = claw;
        this.outtake = outtake;

        updateActions = new UpdateActions(elevator, intake, claw, outtake, extension);
        depositActions = new DepositActions(elevator, intake, claw, outtake, extension);
        actionHelper = new ActionHelper();
        placePurpleActions = new PlacePurpleActions(intake, extension, claw);

    }

    private void moveElevatorByTrajUpdated(int elevatorTarget) {
        elevator.setTarget(elevatorTarget);
        elevator.setPidControl();
    }

    private void closeElevator() {
        outtake.setAngle(Outtake.Angle.INTAKE);
        elevator.setTarget(0);
        elevator.setPidControl();
    }


    public class PlacePurplePixel implements Action {

        ParallelAction placePurple;
        SequentialAction wait;

        public PlacePurplePixel() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            placePurple = new ParallelAction(
                    placePurpleActions.moveIntake(Intake.Angle.INTAKE),
                    placePurpleActions.openExtension(640),
                    wait = new SequentialAction(
                            new SleepAction(500),
                            placePurpleActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH))
            );

            return false;
        }
    }

    public class IntakePixel implements Action {

        ParallelAction intakeWhitePixel;
        SequentialAction waitIntakePixel;

        Intake.Angle angle;

        public IntakePixel(Intake.Angle angle) {
            this.angle = angle;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            intakeWhitePixel = new ParallelAction(
                    placePurpleActions.moveIntake(angle),
                    placePurpleActions.openExtension(750),
                    waitIntakePixel = new SequentialAction(
                            new SleepAction(500),
                            placePurpleActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH))
            );

            return false;

        }
    }


    public class ReadyForPlacement implements Action {

        ParallelAction readyToPlace;
        SequentialAction waitReadyToPlace;

        int elevatorTarget;

        public ReadyForPlacement(int elevatorTarget) {
            this.elevatorTarget = elevatorTarget;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            readyToPlace = new ParallelAction(
                    waitReadyToPlace = new SequentialAction(
                            placePurpleActions.moveIntake(Intake.Angle.MID)),
                    depositActions.moveOuttake(Outtake.Angle.OUTTAKE),
                    depositActions.moveElevator(elevatorTarget)
            );

            return false;

        }
    }

    public class PlaceAndRetract implements Action {

        ParallelAction placePixelsAndRetract;
        SequentialAction waitPlaceOnBoard;


        public PlaceAndRetract() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            placePixelsAndRetract = new ParallelAction(
                    placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                    depositActions.moveElevator(0),
                    waitPlaceOnBoard = new SequentialAction(
                            new SleepAction(500),
                            placePurpleActions.moveIntake(Intake.Angle.OUTTAKE))

            );

            return false;

        }

    }
        public Action placePurplePixel() { return new PlacePurplePixel(); }

        public Action intakePixel (Intake.Angle thisAngle) { return new IntakePixel(thisAngle); }

        public Action readyForPlacement(int chooseTarget) { return new ReadyForPlacement(chooseTarget); }

        public Action placeAndRetract () { return new PlaceAndRetract(); }
}
