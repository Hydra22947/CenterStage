package org.firstinspires.ftc.teamcode.auto.Testing;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.Actions.FasterActions;
import org.firstinspires.ftc.teamcode.auto.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineRedLeft;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "BetterActionsTest")
public class BetterActionsTest extends LinearOpMode {

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
    PlacePurpleActions placePurpleActions;
    UpdateActions updateActions;
    FasterActions fasterActions;

    public static double fixPoseIntakeY = 0;

    enum PropLocation {
        LEFT,
        MIDDLE,
        RIGHT
    }

    PropLocation propLocation = PropLocation.MIDDLE;
    PropPipelineRedLeft propPipelineRedLeft;
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        propPipelineRedLeft = new PropPipelineRedLeft();
        robot.init(hardwareMap, telemetry, autoConstants.startPoseRedLeft);

        autoConstants = new AutoConstants();

        elevator = new Elevator(true);
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension(true);

        intakeExtension.setAuto(true);
        elevator.setAuto(true);

        depositActions = new DepositActions(elevator, intake, claw, outtake, intakeExtension);
        placePurpleActions = new PlacePurpleActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);
        fasterActions = new FasterActions(elevator, intake, claw, outtake, intakeExtension);


        // I kept these actions just in case something goes wrong , if theres something wrong with the faster actions
        // head over to FasterActions and try to solve the issue.

        SequentialAction depositRedMiddle = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.MID),
                depositActions.readyForDeposit(1100),
                placePurpleActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(0.5),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 600)
        );
        SequentialAction transferRedMiddle = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                placePurpleActions.moveClaw(Claw.ClawState.OPEN, ClawSide.RIGHT),
                placePurpleActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(.5),
                placePurpleActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );
        SequentialAction placePurplePixelCloseRedMiddle = new SequentialAction(

                new SleepAction(1.5),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(1),
                placePurpleActions.moveIntake(Intake.Angle.MID),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction retractDepositRedMiddle = new SequentialAction(
                depositActions.retractDeposit()
        );


        SequentialAction intakePixelRedMiddle = new SequentialAction(
                placePurpleActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                placePurpleActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                new SleepAction(.7),
                placePurpleActions.openExtension(750),
                new SleepAction(1),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(0.5),
                placePurpleActions.moveStack(),
                placePurpleActions.closeExtension()

        );
        SequentialAction readyIntakeRedMiddle = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.INTAKE)
        );


        Action trajRedMiddle =
                robot.drive.actionBuilder(robot.drive.pose)

                        .setTangent(Math.toRadians(-70))
                        .lineToY(-12)
                        .setTangent(0)
                        .strafeTo(new Vector2d(-33, -10.5))
                        .stopAndAdd(fasterActions.placePurplePixel())
                        .waitSeconds(.2)
                        .strafeToLinearHeading(new Vector2d(-38.5, -11.5), Math.toRadians(0))

                        //Park - Corner
                        //.lineToY(64)
                        .build();

        Action goForPlacement = robot.drive.actionBuilder(robot.drive.pose)
                .waitSeconds(.5)
                .stopAndAdd(fasterActions.intakePixel(Intake.Angle.TOP_5))
                .waitSeconds(5)
                .stopAndAdd(transferRedMiddle)

                .stopAndAdd(readyIntakeRedMiddle)
                .strafeToLinearHeading(new Vector2d(30, -9), Math.toRadians(0))
                .afterDisp(0.9, fasterActions.readyForPlacement(1100))
                .splineToLinearHeading(new Pose2d(51.55, -32.5, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .stopAndAdd(fasterActions.placeAndRetract())
                .waitSeconds(.5)
                .setTangent(Math.toRadians(90))
                //Park - Close to other board
                .lineToY(-10)
                .turnTo(Math.toRadians(90))
                .build();


        while (opModeInInit() && !isStopRequested()) {
            intake.setAngle(Intake.Angle.MID);
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);

            waitForStart();

            if (isStopRequested()) return;

            runBlocking(trajRedMiddle);
            runBlocking(goForPlacement);


        }
    }
}
