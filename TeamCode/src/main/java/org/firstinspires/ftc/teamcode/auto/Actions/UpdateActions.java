package org.firstinspires.ftc.teamcode.auto.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ClawSide;

public class UpdateActions {


    private LiftSubsystem elevator;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    private Claw claw;
    private IntakeExtensionSubsystem intakeExtension;
    RobotHardware robot = RobotHardware.getInstance();

    int counter = 0;
    public UpdateActions(LiftSubsystem elevator, IntakeSubsystem intake, Claw claw, OuttakeSubsystem outtake, IntakeExtensionSubsystem intakeExtension) {
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
            intake.updateState(intake.getClawStateLeft(), ClawSide.LEFT);
            intake.updateState(intake.getClawStateRight(), ClawSide.RIGHT);
            intake.move(intake.getAngle());
            claw.update();
            outtake.update();
            robot.drive.updatePoseEstimate();

            return true;
        }
    }

    public Action updateSystems()
    {
        return new UpdateSystems();
    }
}
