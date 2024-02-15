package org.firstinspires.ftc.teamcode.auto.Actions;

import static org.firstinspires.ftc.teamcode.auto.Actions.ActionHelper.activateSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.Stopwatch;

public class PlacePurpleActions {

    private Intake intake;
    private IntakeExtension intakeExtension;

    public enum Length {
        EXTENSION_CLOSED,
        ALMOST_HALF,
        HALF,
        FULL
    }

    public enum OpenClaw {
        LEFT_OPEN,
        RIGHT_OPEN,
        BOTH_OPEN
    }


    public enum CloseClaw {
        LEFT_CLOSE,
        RIGHT_CLOSE,
        BOTH_CLOSE
    }

    OpenClaw openClaw;
    CloseClaw closeClaw;
    Length length;


    public PlacePurpleActions(Intake intake, IntakeExtension intakeExtension) {
        this.intake = intake;
        this.intakeExtension = intakeExtension;
        openClaw = OpenClaw.LEFT_OPEN;
        closeClaw = CloseClaw.BOTH_CLOSE;
        this.length = Length.EXTENSION_CLOSED;

    }

    public class PlacePurpleMid implements Action {
        Stopwatch placePurpleMidTimer;

        public PlacePurpleMid() {
            placePurpleMidTimer = new Stopwatch();
            placePurpleMidTimer.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return activateSystem(placePurpleMidTimer, () -> intake.move(Intake.Angle.INTAKE), 0);
        }
    }


    public class Release implements Action {
        Stopwatch releaseTimer;

        public Release(OpenClaw release) {
            releaseTimer = new Stopwatch();
            releaseTimer.reset();
            openClaw = release;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            switch (openClaw) {
                case LEFT_OPEN:
                    return activateSystem(releaseTimer, () -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT), 5000);
                case RIGHT_OPEN:
                    return activateSystem(releaseTimer, () -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.RIGHT), 5000);
                default:
                    return activateSystem(releaseTimer, () -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH), 5000);
            }

        }

    }


    public class Lock implements Action {
        Stopwatch lockTimer;

        public Lock(CloseClaw close) {
            lockTimer = new Stopwatch();
            closeClaw = close;
            lockTimer.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            switch (closeClaw) {
                case LEFT_CLOSE:
                    return activateSystem(lockTimer, () -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT), 500);
                case RIGHT_CLOSE:
                    return activateSystem(lockTimer, () -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.RIGHT), 500);
                default:
                    return activateSystem(lockTimer, () -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH), 500);

            }
        }


    }


    private void moveExtension () {

        switch (length) {
            case EXTENSION_CLOSED:
                intakeExtension.setTarget(0);
                intakeExtension.setPidControl();
                break;
            case ALMOST_HALF:
                intakeExtension.setTarget(650);
                intakeExtension.setPidControl();
                break;
            case HALF:
                intakeExtension.setTarget(820);
                intakeExtension.setPidControl();
                break;
            case FULL:
                intakeExtension.setTarget(1640);
                intakeExtension.setPidControl();
                break;

        }
    }
    public class OpenExtension implements Action {

        Stopwatch openExtensionTimer;

        public OpenExtension(Length currentLength) {
            length = currentLength;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveExtension();
                return false;
            }
    }

    public class CloseExtension implements Action {


        public CloseExtension() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeExtension.setTarget(0);
            return false;
        }
    }


    public class MoveIntake implements Action {
        Intake.Angle angle;
        public MoveIntake(Intake.Angle angle) {
            this.angle = angle;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.move(angle);
            return false;
        }

    }

    public Action placePurpleMid() {
        return new PlacePurpleMid();
    }

    public Action moveIntake(Intake.Angle angle) {
        return new MoveIntake(angle);
    }


    public Action release(OpenClaw openClaw) {

        return new Release(openClaw);
    }

    public Action lock(CloseClaw closeClaw) {
        return new Lock(closeClaw);
    }

    public Action openExtension(Length currentLength) {
        return new OpenExtension(currentLength);
    }

    public Action closeExtension() {
        return new CloseExtension();
    }


}

