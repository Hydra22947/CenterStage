package org.firstinspires.ftc.teamcode.auto;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.OldAuto.AutoLeftBlue;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineBlueLeft;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.openftc.easyopencv.OpenCvWebcam;

public class MTISubmissionAuto extends LinearOpMode {

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

    public enum PropLocation {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static AutoLeftBlue.PropLocation propLocation = AutoLeftBlue.PropLocation.MIDDLE;
    PropPipelineBlueLeft propPipelineBlueLeft;
    OpenCvWebcam webcam;

    public static int tempHeight = 1100;

    @Override
    public void runOpMode() {
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        propPipelineBlueLeft = new PropPipelineBlueLeft();
        robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueLeft);

        autoConstants = new AutoConstants();

        webcam.setPipeline(propPipelineBlueLeft);

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


        SequentialAction intake54 = new SequentialAction(
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                intakeActions.moveIntake(Intake.Angle.TOP_54_AUTO),
                new SleepAction(1),
                intakeActions.openExtension(1200),
                new SleepAction(.5),
                intakeActions.moveIntakeClaw(Intake.ClawState.CLOSE, ClawSide.BOTH),
                new SleepAction(.2),
                intakeActions.moveStack(),
                intakeActions.closeExtension()
                );
        SequentialAction depositBlueMiddle = new SequentialAction(

                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 600),
                new SleepAction(0.5),
                depositActions.readyForDeposit(tempHeight + 150),
                depositActions.retractDeposit()
        );

        SequentialAction placePurplePixelBlueMiddle = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(0.5),
                intakeActions.openExtension(790),
                new SleepAction(1.5),
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
                intakeActions.closeExtension(),
                new SleepAction(0.1),
                intakeActions.moveIntake(Intake.Angle.MID),
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );
        SequentialAction transferPixels = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.LEFT),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.75),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );
        SequentialAction retractDepositBlueMiddle = new SequentialAction(
                depositActions.retractDeposit()
        );


        Action trajBlueMiddle =
                robot.drive.actionBuilder(robot.drive.pose)
                        .stopAndAdd(depositActions.readyForDeposit(tempHeight))
                        .splineToLinearHeading(new Pose2d(40, 26, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixelBlueMiddle)
                        .setTangent(0)
                        .waitSeconds(.2)
                        .stopAndAdd(intakeActions.moveIntake(Intake.Angle.MID))
                        .waitSeconds(.2)
                        .stopAndAdd(intakeActions.closeExtension())

                        //Place Preload on board
                        .splineToLinearHeading(new Pose2d(52.25, 34, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(.1)
                        .stopAndAdd(depositBlueMiddle)

                        .waitSeconds(0.5)
                        .stopAndAdd(retractDepositBlueMiddle)

                        //Intake 54
                        .strafeToLinearHeading(new Vector2d(-20, 36), Math.toRadians(0))
                        .stopAndAdd(intake54)
                        .waitSeconds(0.5)
                        .afterDisp(10,transferPixels)
                        .strafeToLinearHeading(new Vector2d(52.25, 33.25), Math.toRadians(0))
                        .waitSeconds(.1)
                        .stopAndAdd(depositBlueMiddle)

                        //Park
                        .setTangent(Math.toRadians(90))
                        .strafeTo(new Vector2d(50, 33))
                        //Park Close To Wall
                        .strafeTo(new Vector2d(45, 60))
                        .stopAndAdd(retractDepositBlueMiddle)
                        .turnTo(Math.toRadians(-90))
                        .build();

        while (opModeInInit() && !isStopRequested()) {
            intake.setAngle(Intake.Angle.MID);

            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addData("POS", propPipelineBlueLeft.getLocation());

            telemetry.addLine("Initialized");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        runBlocking(new ParallelAction(
                trajBlueMiddle,
                updateActions.updateSystems()
        ));


        while (opModeIsActive()) {
            robot.drive.updatePoseEstimate();
        }

    }

}
