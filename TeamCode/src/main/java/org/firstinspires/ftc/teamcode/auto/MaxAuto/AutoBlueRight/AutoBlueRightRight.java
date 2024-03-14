package org.firstinspires.ftc.teamcode.auto.MaxAuto.AutoBlueRight;

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
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.MaxAuto.Auto;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class AutoBlueRightRight extends Auto {

    private final RobotHardware robot = RobotHardware.getInstance();
    ElapsedTime time;

    // subsystems
    private Elevator elevator;
    private Intake intake;
    private Outtake outtake;
    private Claw claw;
    private IntakeExtension intakeExtension;
    private AutoConstants autoConstants;

    UpdateActions updateActions;

    BlueRightSubsystemActions subsystemActions;
    public SequentialAction blueLeftMiddle;

    public AutoBlueRightRight(Telemetry telemetry, HardwareMap hardwareMap, Intake intake, IntakeExtension intakeExtensiom, Outtake outtake, Claw claw, Elevator elevator) {
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueLeft);

        autoConstants = new AutoConstants();

        this.elevator = elevator;
        this.outtake = outtake;
        this.claw = claw;
        this.intake = intake;
        this.intakeExtension = intakeExtensiom;

        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);
        subsystemActions = new BlueRightSubsystemActions(intake, intakeExtension, outtake, claw, elevator);

        //Trajectories

        Action placePurpleTraj = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-50, 44), Math.toRadians(-90))
                .build();

        Action intake5Traj = robot.drive.actionBuilder(new Pose2d(-34.5, 34, Math.toRadians(0)))
                .strafeToSplineHeading(new Vector2d(-42, 45), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-38, 12, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-30, 9.75), Math.toRadians(0))
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-36.75, 11), Math.toRadians(0))
                .waitSeconds(.5)
                .build();

        Action depositPreloadTraj = robot.drive.actionBuilder(new Pose2d(-53.5, 26, Math.toRadians(0)))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-44.25, 10), Math.toRadians(0))


                //deposit
                .strafeToLinearHeading(new Vector2d(30.25, 9.5), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(52.75, 29, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .waitSeconds(.25)
                .splineToLinearHeading(new Pose2d(52.75, 25.5, Math.toRadians(-5)), Math.toRadians(0)).setTangent(0)
                .waitSeconds(.25)

                .build();
        Action parkTraj = robot.drive.actionBuilder(new Pose2d(52, 32, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(46, 32), Math.toRadians(-90))
                .build();

        ParallelAction placePurplePixel = new ParallelAction(
                placePurpleTraj
        );


        ParallelAction intake54 = new ParallelAction(
                intake5Traj,
                subsystemActions.intake5Action
        );

        ParallelAction depositPreload = new ParallelAction(
                depositPreloadTraj,
                subsystemActions.depositSecondCycle
        );

        ParallelAction park = new ParallelAction(
                parkTraj,
                subsystemActions.retractDeposit
        );

        blueLeftMiddle = new SequentialAction(
                placePurplePixel,
                intake54,
                depositPreload,
                park
        );
    }

    @Override
    public void run() {
        runBlocking(new ParallelAction(
                blueLeftMiddle,
                updateActions.updateSystems()
        ));
    }

}
