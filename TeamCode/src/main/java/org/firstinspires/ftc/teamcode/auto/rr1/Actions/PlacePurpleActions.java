package org.firstinspires.ftc.teamcode.auto.rr1.Actions;

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
    private Stopwatch timer;

    public enum Length {
        EXTENSION_CLOSED,
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
        timer = new Stopwatch();

        openClaw = OpenClaw.LEFT_OPEN;
        closeClaw = CloseClaw.BOTH_CLOSE;
        this.length = Length.EXTENSION_CLOSED;

    }

    private boolean activateSystem(Runnable systemFunction, long delay, Object... parameters) {
        if (timer.hasTimePassed(delay)) {
            systemFunction.run();
            timer.reset();
            return false; // Activation successful
        } else {
            return true; // Activation failed
        }
    }

    public class PlacePurpleMid implements Action {

        public PlacePurpleMid() {
            timer.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            return activateSystem(() -> intake.move(Intake.Angle.INTAKE), 0);

        }
    }


    public class Release implements Action {

        public Release(OpenClaw release) {
            timer.reset();
            openClaw = release;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            switch (openClaw) {
                case LEFT_OPEN:
                    return activateSystem(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT), 500);

                case RIGHT_OPEN:
                    return activateSystem(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.RIGHT), 500);

                case BOTH_OPEN:

                    return activateSystem(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH), 500);

                default:
                    return activateSystem(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH), 500);


            }

        }

    }


    public class Lock implements Action {

        public Lock(CloseClaw close) {

            closeClaw = close;
            timer.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            switch (closeClaw) {
                case LEFT_CLOSE:
                    return activateSystem(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT), 500);

                case RIGHT_CLOSE:
                    return activateSystem(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.RIGHT), 500);

                case BOTH_CLOSE:
                    return activateSystem(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH), 500);

                default:
                    return activateSystem(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH), 500);

            }
        }


    }


    private void moveExtension () {

        switch (length) {
            case EXTENSION_CLOSED:

                intakeExtension.setTarget(0);
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

        public OpenExtension(Length currentLength) {
            length = currentLength;
            timer.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                return activateSystem(()-> moveExtension(), 0 );

            }




        }


    public class Retract implements Action {

        public Retract() {
            timer.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.move(Intake.Angle.INTAKE);
            return false;
        }

    }

    public Action placePurpleMid() {
        return new PlacePurpleMid();
    }

    public Action retract() {
        return new Retract();
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


}

