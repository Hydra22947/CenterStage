package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Hand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.IntakePivot;
import org.firstinspires.ftc.teamcode.util.values.ClawSide;
import org.firstinspires.ftc.teamcode.util.values.Globals;

public class Markers {

    // TODO: add custom actions as soon as possible

    Elevator elevator;
    Claw claw;
    Hand hand;
    Gamepad gamepad2;

    RobotHardware robot;

    Intake intake;

    IntakePivot pivot;

    IntakeExtension intakeExtension;

    ClawSide clawSide;

    double elevatorTarget = Globals.START_ELEVATOR;

    public Markers()
    {

        elevator = new Elevator(gamepad2);
        claw = new Claw();
        hand = new Hand();
        intake = new Intake();
        pivot = new IntakePivot();
        intakeExtension = new IntakeExtension();

        robot = RobotHardware.getInstance();

        robot.addSubsystem(elevator, claw, hand, intake,pivot , intakeExtension);


    }

    public class actions implements Action {

        public void score(){
            new SequentialCommandGroup();
        }

        public void goBack()
        {
            new SequentialCommandGroup();
        };

        public void intakeAndLock()
        {
            new SequentialCommandGroup();
        };




        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }


    public boolean loop(TelemetryPacket packet) {


        return false;
    }






    public Action score () {
      return score();

    }
    public Action goBack () {
        return goBack();

    }

    public Action intakeAndLock () {
        return intakeAndLock();

    }
}







