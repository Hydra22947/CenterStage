package org.firstinspires.ftc.teamcode.auto.rr1.Actions;

import android.view.accessibility.AccessibilityNodeInfo;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.Stopwatch;

public class PlacePurpleActions {

    private Intake intake;
    private IntakeExtension intakeExtension;
    private Stopwatch timer;

    public enum OpenClaw {
        LEFT,
        RIGHT,
        BOTH
    }

    OpenClaw openClaw;

    public PlacePurpleActions(Intake intake, IntakeExtension intakeExtension) {
        this.intake = intake;
        this.intakeExtension = intakeExtension;
        timer = new Stopwatch();

        openClaw = OpenClaw.LEFT;

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


            return activateSystem(()-> intake.move(Intake.Angle.INTAKE), 0);

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
                case LEFT:
                    return activateSystem(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT), 500);

                case RIGHT:
                    return activateSystem(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.RIGHT), 500);

                default:
                    return activateSystem(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH), 500);
            }

        }
    }
    public class Retract implements Action {

        public Retract() {
            //timer.reset();
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

    public Action release(OpenClaw openClaw)
    {

        return new Release(openClaw);
    }
}



