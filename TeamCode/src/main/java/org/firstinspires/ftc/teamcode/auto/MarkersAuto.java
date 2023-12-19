package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

public class MarkersAuto {

    private final RobotHardware robot = RobotHardware.getInstance();

    public static long delay = 1000, elevatorMID , base = 0;
    Outtake outtake;
    Elevator elevator;
    Claw claw;
    Intake intake;
    IntakeExtension intakeExtension;

    public MarkersAuto(Outtake outtake , Elevator elevator,  Claw claw, IntakeExtension intakeExtension, Intake intake)
    {
        this.outtake = outtake;
        this.elevator = elevator;
        this.claw = claw;
        this.intakeExtension = intakeExtension;
        this.intake = intake;

    }


    MarkerCallback placePurplePixelMarker = new MarkerCallback() {
        @Override
        public void onMarkerReached()
        {
            intake.move(Intake.Angle.INTAKE);
            intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT);
            intake.move(Intake.Angle.TOP_54);

        }
    };

    MarkerCallback openIntake= new MarkerCallback() {
        @Override
        public void onMarkerReached() {
            intakeExtension.openExtension();
            claw.updateState(Claw.ClawState.OPEN , ClawSide.BOTH);
            intake.move(Intake.Angle.INTAKE);



        }
    };

    MarkerCallback closeIntake= new MarkerCallback() {
        @Override
        public void onMarkerReached() {

            claw.updateState(Claw.ClawState.CLOSED , ClawSide.BOTH);
            intakeExtension.closeExtension();
            intake.move(Intake.Angle.TRANSFER);
            claw.updateState(Claw.ClawState.OPEN , ClawSide.BOTH);

        }
    };

    MarkerCallback openElevatorHIGH = new MarkerCallback() {
        @Override
        public void onMarkerReached() {

            intake.move(Intake.Angle.MID);
            elevator.setTarget(Elevator.MAX_LEVEL);
            elevator.update();


        }
    };

    MarkerCallback openElevatorMID = new MarkerCallback() {
        @Override
        public void onMarkerReached() {

            intake.move(Intake.Angle.MID);
            elevator.setTarget(elevatorMID);
            elevator.update();

        }
    };

    MarkerCallback openElevatorLOW = new MarkerCallback() {
        @Override
        public void onMarkerReached() {

            intake.move(Intake.Angle.MID);
            elevator.setTarget(Elevator.BASE_LEVEL);
            elevator.update();

        }
    };


    MarkerCallback closeElevator = new MarkerCallback() {
        @Override
        public void onMarkerReached() {

            outtake.setAngle(Outtake.Angle.OUTTAKE);
            elevator.setTarget(base);
            elevator.update();
            intake.move(Intake.Angle.MID);

        }
    };


    MarkerCallback park = new MarkerCallback() {
        @Override
        public void onMarkerReached() {
            intake.move(Intake.Angle.TRANSFER);
        }
    };
}
