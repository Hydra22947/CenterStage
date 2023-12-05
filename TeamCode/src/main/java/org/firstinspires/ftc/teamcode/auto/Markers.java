package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.Globals;

public class Markers {


    long timer = 5000;
    Elevator elevator;
    Claw claw;
    Gamepad gamepad2;

    RobotHardware robot;

    Intake intake;

//    IntakeExtension intakeExtension;

    Outtake outtake;


    ClawSide clawSide;


    public Markers()
    {

        elevator = new Elevator(gamepad2);
//        claw = new Claw();
        intake = new Intake();
//        intakeExtension = new IntakeExtension(null);
        outtake = new Outtake();

        robot = RobotHardware.getInstance();

        //robot.addSubsystem(elevator, claw, intake , intakeExtension);


    }

    public class actions implements Action {


        public void score()
        {
            new SequentialCommandGroup(
                    new IntakeCommand(intake, Intake.Angle.INTAKE),
                    new ElevatorCommand(elevator, Globals.START_ELEVATOR ),
                    new OuttakeCommand(outtake , Outtake.Angle.OUTTAKE),
                    //////////////////////////////////////////////////////
                    new WaitCommand(timer),
                    //////////////////////////////////////////////////////
                    new OuttakeCommand(outtake , Outtake.Angle.INTAKE),
                    new ElevatorCommand(elevator, 0),
                    new IntakeCommand(intake , Intake.Angle.TRANSFER));

        }


        public void intakeRelease() { new IntakeCommand(intake , Intake.Angle.TRANSFER); }

        public void intakeAndLock()
        {
            new SequentialCommandGroup(
//                    new IntakeExtensionCommand( intakeExtension , IntakeExtension.ExtensionState.OPEN),
                    new IntakeCommand(intake , Intake.Angle.INTAKE),
                    //////////////////////////////////////////////////////
                    new WaitCommand(timer),
                    //////////////////////////////////////////////////////
                    new IntakeCommand(intake , Intake.Angle.TRANSFER)
//                    new IntakeExtensionCommand( intakeExtension , IntakeExtension.ExtensionState.CLOSE)
            );
        };




        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return true;
        }
    }


    public boolean loop(TelemetryPacket packet) {


        return true;
    }


    public Action score () { return score() ;

    }
    public Action goBack () { return goBack(); }

    public Action intakeAndLock () { return intakeAndLock(); }

    public Action intakeRelease() { return intakeRelease(); }
}







