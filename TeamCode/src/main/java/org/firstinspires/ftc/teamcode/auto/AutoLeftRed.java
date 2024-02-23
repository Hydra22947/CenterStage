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
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineRedLeft;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "2+0 - Auto Red Left")
public class AutoLeftRed extends LinearOpMode {
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

    enum PropLocation
    {
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

        initCamera();
        webcam.setPipeline(propPipelineRedLeft);

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

        SequentialAction depositRedLeft = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.MID),
                depositActions.readyForDeposit(1100),
                placePurpleActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(0.5),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 600)
        );
        SequentialAction transferRedLeft = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                placePurpleActions.moveClaw(Claw.ClawState.OPEN, ClawSide.RIGHT),
                placePurpleActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(.5),
                placePurpleActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );
        SequentialAction placePurplePixelCloseRedLeft = new SequentialAction(

                new SleepAction(1.5),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(1),
                placePurpleActions.moveIntake(Intake.Angle.MID),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction retractDepositRedLeft = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction intakePixelRedLeft = new SequentialAction(
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
        SequentialAction readyIntakeRedLeft = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.INTAKE)
        );

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


        SequentialAction depositRedRight = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.MID),
                depositActions.readyForDeposit(1100),
                placePurpleActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(0.5),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 600)
        );
        SequentialAction transferRedRight = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                placePurpleActions.moveClaw(Claw.ClawState.OPEN, ClawSide.RIGHT),
                placePurpleActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(.5),
                placePurpleActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );
        SequentialAction placePurplePixelCloseRedRight = new SequentialAction(

                new SleepAction(1.5),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(1),
                placePurpleActions.moveIntake(Intake.Angle.MID),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction retractDepositRedRight = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction intakePixelRedRight = new SequentialAction(
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
        SequentialAction readyIntakeRedRight = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.INTAKE)
        );

        Action trajRedLeft =
                robot.drive.actionBuilder(robot.drive.pose)

                        .strafeToLinearHeading(new Vector2d(-32, -28), Math.toRadians(0))
                        .waitSeconds(.2)
                        .lineToX(-48)
                        .lineToX(-32)
                        .stopAndAdd(readyIntakeRedLeft)
                        .stopAndAdd(placePurplePixelCloseRedLeft)
                        .strafeToLinearHeading(new Vector2d(-40,-11.25), Math.toRadians(0))
                        .waitSeconds(.5)
                        .stopAndAdd(intakePixelRedLeft)
                        .waitSeconds(5)
                        .stopAndAdd(transferRedLeft)

                        .stopAndAdd(readyIntakeRedLeft)
                        .strafeToLinearHeading(new Vector2d(30, -9), Math.toRadians(0))
                        .stopAndAdd(placePurpleActions.moveIntake(Intake.Angle.MID))
                        .stopAndAdd(depositActions.readyForDeposit(1300))
                        .splineToLinearHeading(new Pose2d(51.55, -30.25, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(depositRedLeft)
                        .waitSeconds(.5)
                        .stopAndAdd(retractDepositRedLeft)
                        .setTangent(Math.toRadians(90))

                        //Park - Close to other board
                        .lineToY(-10)
                        .turnTo(Math.toRadians(90))

                        //Park - Corner
                        //.lineToY(64)
                        .build();

        Action trajRedMiddle =
                robot.drive.actionBuilder(robot.drive.pose)

                        .setTangent(Math.toRadians(-70))
                        .lineToY(-12)
                        .stopAndAdd(readyIntakeRedMiddle)
                        .setTangent(0)
                        .strafeTo(new Vector2d(-33, -10.5))
                        .stopAndAdd(placePurplePixelCloseRedMiddle)
                        .waitSeconds(.2)
                        .strafeToLinearHeading(new Vector2d(-38.5, -11.5), Math.toRadians(0))
                        .waitSeconds(.5)
                        .stopAndAdd(intakePixelRedMiddle)
                        .waitSeconds(5)
                        .stopAndAdd(transferRedMiddle)

                        .stopAndAdd(readyIntakeRedMiddle)
                        .strafeToLinearHeading(new Vector2d(30, -9), Math.toRadians(0))
                        .afterDisp(0.9, depositActions.readyForDeposit(1100))
                        .afterDisp(1, placePurpleActions.moveIntake(Intake.Angle.MID))
                        .splineToLinearHeading(new Pose2d(51.55, -32.5, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(depositRedMiddle)
                        .waitSeconds(.5)
                        .setTangent(Math.toRadians(90))
                        .stopAndAdd(retractDepositRedMiddle)
                        //Park - Close to other board
                        .lineToY(-10)
                        .turnTo(Math.toRadians(90))

                        //Park - Corner
                        //.lineToY(64)
                        .build();

        Action trajRedRight =
                robot.drive.actionBuilder(robot.drive.pose)
                        .setTangent(Math.toRadians(-80))
                        .lineToYLinearHeading(-30, Math.toRadians(-180))
                        .stopAndAdd(readyIntakeRedRight)
                        .setTangent(0)
                        .lineToX(-36.4)
                        .stopAndAdd(placePurplePixelCloseRedRight)
                        .waitSeconds(.2)

                        .strafeToLinearHeading(new Vector2d(-33, -11.5),Math.toRadians(0))
                        .waitSeconds(.5)
                        .stopAndAdd(intakePixelRedRight)
                        .waitSeconds(5)
                        .stopAndAdd(transferRedRight)

                        .stopAndAdd(readyIntakeRedRight)
                        .strafeToLinearHeading(new Vector2d(30, -9), Math.toRadians(0))
                        .afterDisp(0.9, depositActions.readyForDeposit(1100))
                        .afterDisp(1, placePurpleActions.moveIntake(Intake.Angle.MID))
                        .splineToLinearHeading(new Pose2d(51.45, -38.25, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(depositRedRight)
                        .waitSeconds(.5)
                        .setTangent(Math.toRadians(90))
                        .stopAndAdd(retractDepositRedRight)
                        //Park - Close to other board
                        .lineToY(-10)
                        .turnTo(Math.toRadians(90))

                        //Park - Corner
                        //.lineToY(64)

                        .build();

        while (opModeInInit() && !isStopRequested()) {
            intake.setAngle(Intake.Angle.MID);
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addData("POS", propPipelineRedLeft.getLocation());
            switch (propPipelineRedLeft.getLocation())
            {
                case Left:
                    propLocation = PropLocation.LEFT;
                    break;
                case Right:
                    propLocation = PropLocation.RIGHT;
                    break;
                case Center:
                    propLocation = PropLocation.MIDDLE;
                    break;
            }
            telemetry.addLine("Initialized");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        switch (propLocation)
        {
            case RIGHT:
                runBlocking(new ParallelAction(
                        trajRedRight,
                        updateActions.updateSystems()
                ));
                break;
            case MIDDLE:
                runBlocking(new ParallelAction(
                        trajRedMiddle,
                        updateActions.updateSystems()
                ));
                break;
            case LEFT:
                runBlocking(new ParallelAction(
                        trajRedLeft,
                        updateActions.updateSystems()
                ));
                break;
        }

        while (opModeIsActive())
        {
            robot.drive.updatePoseEstimate();
        }

        writeToFile(robot.drive.pose.heading.log());
    }

    void initCamera()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }


}