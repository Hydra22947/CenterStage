package org.firstinspires.ftc.teamcode.auto.MaxAuto.AutoBlueLeft;

import static org.firstinspires.ftc.teamcode.auto.MaxAuto.AutoBlueLeft.BlueLeftSubsystemActions.closeIntakeWhitePixelAction;
import static org.firstinspires.ftc.teamcode.auto.MaxAuto.AutoBlueLeft.BlueLeftSubsystemActions.depositBlue;
import static org.firstinspires.ftc.teamcode.auto.MaxAuto.AutoBlueLeft.BlueLeftSubsystemActions.depositSecondCycle;
import static org.firstinspires.ftc.teamcode.auto.MaxAuto.AutoBlueLeft.BlueLeftSubsystemActions.openIntakeWhitePixelAction;
import static org.firstinspires.ftc.teamcode.auto.MaxAuto.AutoBlueLeft.BlueLeftSubsystemActions.placePurplePixelSequence;

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
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;


public class AutoBlueLeftRight {


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

    public SequentialAction blueLeftRight;

    public AutoBlueLeftRight(Telemetry telemetry, HardwareMap hardwareMap) {
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueLeft);

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


        //Trajectories
        Action placePurpleTraj = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(34.7,  32), Math.toRadians(0))
                .build();

        Action placeYellowPixelTraj = robot.drive.actionBuilder(new Pose2d(34.7,  32, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(50.25, 34, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Action intake54Traj = robot.drive.actionBuilder(new Pose2d(50.25, 34, Math.toRadians(0)))
                //Going to intake position
                .setTangent(Math.toRadians(-120))
                .splineToConstantHeading(new Vector2d(30, 9.5), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-28, 10.84, Math.toRadians(0)), Math.toRadians(180))
                .waitSeconds(.25)

                //Getting Closer and fixing angle
                .strafeToLinearHeading(new Vector2d(-33, 10.84), Math.toRadians(0))
                .build();

        Action place54Traj = robot.drive.actionBuilder(new Pose2d(-33, 10.84, Math.toRadians(0)))

                .strafeToLinearHeading(new Vector2d(30, 9), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51, 25, Math.toRadians(5)), Math.toRadians(0))
                .build();

        ParallelAction placePurplePixel = new ParallelAction(
                placePurpleTraj,
                placePurplePixelSequence
        );

        ParallelAction placePreloadOnBoard = new ParallelAction(
                placeYellowPixelTraj,
                depositBlue
        );

        ParallelAction intake54 = new ParallelAction(
                intake54Traj,
                openIntakeWhitePixelAction,
                closeIntakeWhitePixelAction
        );

        ParallelAction deposit54 = new ParallelAction(
                place54Traj,
                depositSecondCycle
        );


        blueLeftRight = new SequentialAction(
                placePurplePixel,
                placePreloadOnBoard,
                intake54,
                deposit54
        );
    }

    public Action run() {
        return new ParallelAction(
                blueLeftRight,
                updateActions.updateSystems()
        );
    }


}
