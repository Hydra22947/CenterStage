package org.firstinspires.ftc.teamcode.auto.Actions;

import static org.firstinspires.ftc.teamcode.auto.Actions.ActionHelper.activateSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.Stopwatch;

public class IntakeActions {

    private Intake intake;
    private IntakeExtension intakeExtension;
    Claw claw;

    public enum Length {
        EXTENSION_CLOSED,
        ALMOST_HALF,
        QUARTER,
        TWO_PLUS_ONE,
        HALF,
        FULL
    }

    public enum OpenClaw {
        LEFT_OPEN,
        RIGHT_OPEN,
        BOTH_OPEN
    }

    public enum FailSafe {
        ACTIVATED,
        DEACTIVATED
    }

    public enum CloseClaw {
        LEFT_CLOSE,
        RIGHT_CLOSE,
        BOTH_CLOSE,

        SUPER_LEFT_CLOSE,
        SUPER_RIGHT_CLOSE,
        SUPER_BOTH_CLOSE
    }

    OpenClaw openClaw;
    CloseClaw closeClaw;
    Length length;
    FailSafe failSafe;

    public IntakeActions(Intake intake, IntakeExtension intakeExtension, Claw claw) {
        this.intake = intake;
        this.claw = claw;
        this.intakeExtension = intakeExtension;
        openClaw = OpenClaw.LEFT_OPEN;
        closeClaw = CloseClaw.BOTH_CLOSE;
        this.length = Length.EXTENSION_CLOSED;
        failSafe = FailSafe.ACTIVATED;

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

    public class FailSafeClaw implements Action {
        Stopwatch releaseTimer;

        public FailSafeClaw(FailSafe clawFailSafe) {
            releaseTimer = new Stopwatch();
            releaseTimer.reset();
            failSafe = clawFailSafe;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            switch (failSafe) {
                case ACTIVATED:
                    return activateSystem(releaseTimer, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH), 5000);
                case DEACTIVATED:
                    return activateSystem(releaseTimer, () -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH), 5000);
                default:
                    return activateSystem(releaseTimer, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH), 5000);
            }

        }

    }



    public class MoveIntakeClaw implements Action {
        Intake.ClawState clawState;
        ClawSide clawSide;

        public MoveIntakeClaw(Intake.ClawState clawState, ClawSide clawSide) {
            this.clawState = clawState;
            this.clawSide = clawSide;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.updateClawState(clawState, clawSide);
            return false;
        }

    }

    public class MoveOuttakeClaw implements Action {
        Claw.ClawState clawState;
        ClawSide clawSide;

        public MoveOuttakeClaw(Claw.ClawState clawState, ClawSide clawSide) {
            this.clawState = clawState;
            this.clawSide = clawSide;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.updateState(clawState, clawSide);
            return false;
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


    private void moveExtension (int length) {
        intakeExtension.setTarget(length);
        intakeExtension.setPidControl();
    }
    public class OpenExtension implements Action {

        int length = 0;

        public OpenExtension(int length) {
            this.length = length;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveExtension(length);
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

    public class MoveStack implements Action {


        public MoveStack() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.moveStack();
            return false;
        }
    }


    public  class MoveIntake implements Action {
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

    public Action moveIntakeClaw(Intake.ClawState clawState, ClawSide clawSide) {
        return new MoveIntakeClaw(clawState, clawSide);
    }

    public Action moveClaw(Claw.ClawState clawState, ClawSide clawSide) {
        return new MoveOuttakeClaw(clawState, clawSide);
    }



    public Action release(OpenClaw openClaw) {

        return new Release(openClaw);
    }

    public Action lock(CloseClaw closeClaw) {
        return new Lock(closeClaw);
    }

    public Action moveStack() {
        return new MoveStack();
    }


    public Action openExtension(int length) {
        return new OpenExtension(length);
    }

    public Action closeExtension() {
        return new CloseExtension();
    }

    public Action failSafeClaw(FailSafe failSafeClaw) { return new FailSafeClaw(failSafeClaw) ; }


}

