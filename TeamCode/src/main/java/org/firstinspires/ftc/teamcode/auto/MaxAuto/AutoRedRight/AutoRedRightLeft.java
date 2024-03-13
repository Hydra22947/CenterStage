package org.firstinspires.ftc.teamcode.auto.MaxAuto.AutoRedRight;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.auto.Actions.SubsystemActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.MaxAuto.Auto;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class AutoRedRightLeft extends Auto {

    private final RobotHardware robot = RobotHardware.getInstance();
    ElapsedTime time;

    // subsystems
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;
    AutoConstants autoConstants;

    DepositActions depositActions;
    PlacePurpleActions intakeActions;
    UpdateActions updateActions;

    SubsystemActions subsystemActions;
    public SequentialAction redRightLeft;

    public AutoRedRightLeft(Telemetry telemetry, HardwareMap hardwareMap) {
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, autoConstants.startPoseRedRight);

        autoConstants = new AutoConstants();

        elevator = new Elevator(true);
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension(true);

        intakeExtension.setAuto(true);
        elevator.setAuto(true);

        depositActions = new DepositActions(elevator, intake, claw, outtake, intakeExtension);
        intakeActions = new PlacePurpleActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);
        subsystemActions = new SubsystemActions(intake, intakeExtension, outtake, claw, elevator);

        //Trajectories
        Action placePurpleTraj = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(48, -34.25), Math.toRadians(0))
                .build();

        Action placeYellowPixelTraj = robot.drive.actionBuilder(new Pose2d(48, -34., Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(50.25, -34, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Action intake54Traj = robot.drive.actionBuilder(new Pose2d(50.25, -34, Math.toRadians(0)))
                //Going to intake position
                .setTangent(Math.toRadians(-120))
                .splineToConstantHeading(new Vector2d(30, -9.5), Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(-28, -10.84, Math.toRadians(0)), Math.toRadians(180))
                .waitSeconds(.25)

                //Getting Closer and fixing angle
                .strafeToLinearHeading(new Vector2d(-33, -10.84), Math.toRadians(0))
                .build();

        Action place54Traj = robot.drive.actionBuilder(new Pose2d(-33, -10.84, Math.toRadians(0)))

                .strafeToLinearHeading(new Vector2d(30, -9), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51, -25, Math.toRadians(-5)), Math.toRadians(0))
                .build();

        ParallelAction placePurplePixel = new ParallelAction(
                placePurpleTraj,
                subsystemActions.placePurplePixelSequence
        );

        ParallelAction placePreloadOnBoard = new ParallelAction(
                placeYellowPixelTraj,
                subsystemActions.depositBlue
        );

        ParallelAction intake54 = new ParallelAction(
                intake54Traj,
                subsystemActions.openIntakeWhitePixelAction,
                subsystemActions.closeIntakeWhitePixelAction
        );

        ParallelAction deposit54 = new ParallelAction(
                place54Traj,
                subsystemActions.depositSecondCycle
        );

        redRightLeft = new SequentialAction(
                placePurplePixel
                //    placePreloadOnBoard,
                //    intake54,
                //   deposit54
        );
    }

    @Override
    public void run() {
        runBlocking(
                new ParallelAction(
                        redRightLeft,
                        updateActions.updateSystems())

        );
    }
}
