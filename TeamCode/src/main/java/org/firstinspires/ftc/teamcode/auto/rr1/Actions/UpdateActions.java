package org.firstinspires.ftc.teamcode.auto.rr1.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.Stopwatch;

public class UpdateActions {


    private Elevator elevator;
    private Intake intake;
    private Outtake outtake;
    private Claw claw;
    private IntakeExtension intakeExtension;

    int counter = 0;
    public UpdateActions(Elevator elevator, Intake intake, Claw claw, Outtake outtake, IntakeExtension intakeExtension) {
        this.elevator = elevator;
        this.intake = intake;
        this.claw = claw;
        this.outtake = outtake;
        this.intakeExtension = intakeExtension;
    }
    public class UpdateSystems implements Action {

        public UpdateSystems() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeExtension.setPidControl();
            elevator.setPidControl();
            intake.updateClawState(intake.getClawStateLeft(), ClawSide.LEFT);
            intake.updateClawState(intake.getClawStateRight(), ClawSide.RIGHT);
            intake.move(intake.getAngle());
            claw.update();
            outtake.update();

            telemetryPacket.addLine(String.valueOf(elevator.getTargetLeft()));

            return true;
        }
    }

    public Action updateSystems()
    {
        return new UpdateSystems();
    }
}
