package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static org.firstinspires.ftc.teamcode.auto.AutoSettings.writeToFile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineBlueLeft;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "2+2 - Auto Blue Left")
@Disabled
public class AutoLeftBlueMax extends LinearOpMode {
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

    enum PropLocation
    {
        LEFT,
        CENTER,
        RIHGT
    }

    public static PropLocation propLocation = PropLocation.CENTER;

    public static int tempHeight = 1450;

    @Override
    public void runOpMode() {
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

        SequentialAction depositBlue = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),
                new SleepAction(0.1),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 0),
                new SleepAction(.2),
                depositActions.moveElevator(1300),
                new SleepAction(.2),
                intakeActions.moveIntake(Intake.Angle.TOP_54_AUTO),
                depositActions.retractDeposit()

        );


        SequentialAction depositSecondCycle = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),

                intakeActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(1),
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),

                new SleepAction(0.1),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),

                new SleepAction(0.8),
                depositActions.moveElevator(tempHeight),
                depositActions.retractDeposit()
        );

        SequentialAction placePurplePixel = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(0.5),
                intakeActions.openExtension(790),
                new SleepAction(0.5),
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
                intakeActions.closeExtension(),
                intakeActions.moveIntake(Intake.Angle.MID),
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction openIntakeWhitePixel = new SequentialAction(
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                intakeActions.openExtension(1000)
        );

        SequentialAction closeIntakeWhitePixel = new SequentialAction(
                new SleepAction(.5),
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(.5),
                intakeActions.moveStack(),
                new SleepAction(.5),
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                intakeActions.closeExtension()
        );


        SequentialAction transfer = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.75),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );


        SequentialAction retractDepositBlueMax = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction readyForDeposit = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),
                new SleepAction(0.1),
                depositActions.readyForDeposit(tempHeight)

        );

        Action trajBlueRight =
                robot.drive.actionBuilder(robot.drive.pose)
                        //Place purple pixel
                        .stopAndAdd(depositActions.readyForDeposit(950))
                        .splineToLinearHeading(new Pose2d(34.7, 32, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixel)
                        .setTangent(0)
                        .waitSeconds(.2)
                        .stopAndAdd(intakeActions.closeExtension())


                        //Place Preload on board
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(51, 29))
                        .stopAndAdd(depositBlue)
                        .waitSeconds(0.3)


                        //Intake 54
                        .waitSeconds(0.2)
                        .setTangent(Math.toRadians(-120))
                        .stopAndAdd(intakeActions.moveIntake(Intake.Angle.TOP_54))
                        .splineToConstantHeading(new Vector2d(30, 9.5), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-28, 10.85, Math.toRadians(0)), Math.toRadians(180))

                        .stopAndAdd(openIntakeWhitePixel)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-33, 10.85), Math.toRadians(0))

                        .stopAndAdd(closeIntakeWhitePixel)
                        .waitSeconds(0.5)
                        .stopAndAdd(transfer)

                        //Deposit
                        .strafeToLinearHeading(new Vector2d(30, 9), Math.toRadians(0))
                        .stopAndAdd(readyForDeposit)
                        .splineToLinearHeading(new Pose2d(51, 34, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(depositSecondCycle)
                        .waitSeconds(0.5)


                        //Park
                        .setTangent(Math.toRadians(90))
                        .strafeTo(new Vector2d(48, 28.5))
                        .stopAndAdd(retractDepositBlueMax)
                        .strafeToLinearHeading(new Vector2d(45, 55), Math.toRadians(-90))
                        .strafeTo(new Vector2d(45.25, 60))
                        .build();


        Action trajBlueMiddle =
                robot.drive.actionBuilder(robot.drive.pose)
                        //Place purple pixel
                        .stopAndAdd(depositActions.readyForDeposit(950))
                        .splineToLinearHeading(new Pose2d(40, 26, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixel)

                        .setTangent(0)
                        .stopAndAdd(intakeActions.moveIntake(Intake.Angle.MID))
                        .stopAndAdd(intakeActions.closeExtension())

                        //Place Preload on board
                        .splineToLinearHeading(new Pose2d(50.25, 34, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(depositBlue)

                        //Intake 54
                        .setTangent(Math.toRadians(-120))
                        .stopAndAdd(intakeActions.moveIntake(Intake.Angle.TOP_54))
                        .splineToConstantHeading(new Vector2d(30, 9.5), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-28, 10.84, Math.toRadians(0)), Math.toRadians(180))

                        .stopAndAdd(openIntakeWhitePixel)
                        .waitSeconds(.7)
                        .strafeToLinearHeading(new Vector2d(-33, 10.84), Math.toRadians(0))

                        .stopAndAdd(closeIntakeWhitePixel)
                        .waitSeconds(0.5)
                        .stopAndAdd(transfer)

                        //Deposit
                        .strafeToLinearHeading(new Vector2d(30, 9), Math.toRadians(0))
                        .afterTime(0.1, readyForDeposit)
                        .splineToLinearHeading(new Pose2d(51, 25, Math.toRadians(5)), Math.toRadians(0))
                        .stopAndAdd(depositSecondCycle)
                        .waitSeconds(0.1)

                        //Park
                        .strafeToLinearHeading(new Vector2d(46, 33.5), Math.toRadians(-90))
                        .stopAndAdd(retractDepositBlueMax)
                        .build();

        Action trajBlueLeft =
                robot.drive.actionBuilder(robot.drive.pose)
                        //Place purple pixel
                        .stopAndAdd(depositActions.readyForDeposit(950))
                        .splineToLinearHeading(new Pose2d(48, 34.25, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixel)

                        //Deposit yellow
                        .setTangent(0)
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(50.75, 40.75))
                        .stopAndAdd(depositBlue)

                        //Intake 54
                        .waitSeconds(0.2)
                        .setTangent(Math.toRadians(-120))
                        .stopAndAdd(intakeActions.moveIntake(Intake.Angle.TOP_54))
                        .splineToConstantHeading(new Vector2d(30, 9.5), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-28, 10.85, Math.toRadians(0)), Math.toRadians(180))

                        .stopAndAdd(openIntakeWhitePixel)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-33, 10.85), Math.toRadians(0))

                        .stopAndAdd(closeIntakeWhitePixel)

                        .waitSeconds(0.5)
                        .stopAndAdd(transfer)

                        //Deposit
                        .strafeToLinearHeading(new Vector2d(30, 9), Math.toRadians(0))
                        .stopAndAdd(readyForDeposit)
                        .splineToLinearHeading(new Pose2d(51, 34, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(depositSecondCycle)
                        .waitSeconds(0.5)

                        //Park
                        .setTangent(Math.toRadians(90))
                        .strafeTo(new Vector2d(47, 40.5))
                        .strafeTo(new Vector2d(45, 60))
                        .stopAndAdd(retractDepositBlueMax)
                        .turnTo(Math.toRadians(-90))
                        .build();


        while (opModeInInit() && !isStopRequested()) {
            intake.setAngle(Intake.Angle.MID);

            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addLine("Initialized");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        switch (propLocation)
        {
            case LEFT:
                runBlocking(new ParallelAction(
                        trajBlueLeft,
                        updateActions.updateSystems()
                ));
                break;
            case CENTER:
                runBlocking(new ParallelAction(
                        trajBlueMiddle,
                        updateActions.updateSystems()
                ));
                break;
            case RIHGT:
                runBlocking(new ParallelAction(
                        trajBlueRight,
                        updateActions.updateSystems()
                ));
                break;
        }

        while (opModeIsActive()) {
            robot.drive.updatePoseEstimate();
        }

        writeToFile(robot.drive.pose.heading.log());
    }


}