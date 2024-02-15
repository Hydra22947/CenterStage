package org.firstinspires.ftc.teamcode.auto.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

public class InitSystems extends CommandBase {
    Elevator elevator;
    Outtake outtake;
    Claw claw;
    Intake intake;
    IntakeExtension intakeExtension;

    public InitSystems(Elevator elevator, Outtake outtake, Claw claw, Intake intake, IntakeExtension intakeExtension)
    {
       this.elevator = elevator;
       this.outtake = outtake;
       this.claw = claw;
       this.intake = intake;
       this.intakeExtension = intakeExtension;
    }

    @Override
    public void execute() {

//        elevator.setTarget(0);
        elevator.setPidControl();

        outtake.setAngle(Outtake.Angle.INTAKE);

        claw.setBothClaw(Claw.ClawState.OPEN);
        claw.update();

        intake.move(Intake.Angle.OUTTAKE);
        intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);

//        intakeExtension.setTarget(0);
        intakeExtension.setPidControl();
    }


}
