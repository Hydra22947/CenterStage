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
    PlacePurpleActions placePurpleActions;
    UpdateActions updateActions;


    public static int tempHeight = 1100;
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

        depositActions = new DepositActions(elevator, intake, claw, outtake , intakeExtension);
        placePurpleActions = new PlacePurpleActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);

        SequentialAction depositBlueMiddle = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.MID),
                new SleepAction(0.5),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD,0),
                new SleepAction(.2),
                placePurpleActions.moveIntake(Intake.Angle.TOP_54_AUTO),
                depositActions.retractDeposit()

        );


        SequentialAction depositSecondCycle = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.MID),

                placePurpleActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(1),
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),

                new SleepAction(0.1),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),

                new SleepAction(0.5),
                depositActions.moveElevator(1100),
                depositActions.retractDeposit()
        );

        SequentialAction placePurplePixelBlueMiddle = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(0.5),
                placePurpleActions.openExtension(790),
                new SleepAction(1.5),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
                placePurpleActions.closeExtension(),
                new SleepAction(0.1),
                placePurpleActions.moveIntake(Intake.Angle.MID),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction openIntakeWhitePixel = new SequentialAction(
                placePurpleActions.openExtension(600),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN)
        );

        SequentialAction closeIntakeWhitePixel = new SequentialAction(
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                placePurpleActions.moveStack(),
                new SleepAction(.5),
                placePurpleActions.moveIntake(Intake.Angle.OUTTAKE),
                placePurpleActions.closeExtension()
        );


        SequentialAction transfer= new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                placePurpleActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                placePurpleActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.75),
                placePurpleActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );


        SequentialAction retractDepositBlueMax = new SequentialAction(
                depositActions.retractDeposit()
        );



        Action trajBlueMiddle =
                robot.drive.actionBuilder(robot.drive.pose)
                        .stopAndAdd(depositActions.readyForDeposit(tempHeight))
                        .splineToLinearHeading(new Pose2d(40 , 26, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixelBlueMiddle)
                        .setTangent(0)
                        .waitSeconds(.2)
                        .stopAndAdd(placePurpleActions.moveIntake(Intake.Angle.MID))
                        .waitSeconds(.2)
                        .stopAndAdd(placePurpleActions.closeExtension())
                        //Place Preload on board
                        .waitSeconds(.2)
                        .splineToLinearHeading(new Pose2d(51, 34, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(.1)
                        .stopAndAdd(depositBlueMiddle)
                        .waitSeconds(0.2)
                        .setTangent(Math.toRadians(-120))
                        .stopAndAdd(placePurpleActions.moveIntake(Intake.Angle.TOP_54))
                        .splineToConstantHeading(new Vector2d(30, 8), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-38.8, 12, Math.toRadians(0)), Math.toRadians(180))
                        .stopAndAdd(placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN))
                        .stopAndAdd(openIntakeWhitePixel)
                        .waitSeconds(1.5)
                        .strafeToLinearHeading(new Vector2d(-40,14),Math.toRadians(0))
                        .stopAndAdd(closeIntakeWhitePixel)
                        .waitSeconds(0.5)
                        .stopAndAdd(transfer)
                        .strafeToLinearHeading(new Vector2d(30, 9), Math.toRadians(0))
                        .stopAndAdd(placePurpleActions.moveIntake(Intake.Angle.MID))
                        .stopAndAdd(depositActions.readyForDeposit(tempHeight))
                        .splineToLinearHeading(new Pose2d(51, 34, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(depositSecondCycle)
                        .waitSeconds(0.5)
                        //Park
                        .strafeTo(new Vector2d(48, 33.5))
                        .stopAndAdd(retractDepositBlueMax)
                        .strafeToLinearHeading(new Vector2d(46, 10), Math.toRadians(-90))
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

        runBlocking(new ParallelAction(
                trajBlueMiddle,
                updateActions.updateSystems()
        ));

        while (opModeIsActive())
        {
            robot.drive.updatePoseEstimate();
        }

        writeToFile(robot.drive.pose.heading.log());
    }


}