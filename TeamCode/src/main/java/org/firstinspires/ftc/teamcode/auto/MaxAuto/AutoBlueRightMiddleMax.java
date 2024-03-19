package org.firstinspires.ftc.teamcode.auto.MaxAuto;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoSettings.writeToFile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
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
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineRedRight;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "2+3 - Auto Right Middle Blue MAX")
public class AutoBlueRightMiddleMax extends LinearOpMode {
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

    public static PropLocation propLocation = PropLocation.MIDDLE;
    PropPipelineRedRight propPipelineRedRight;
    OpenCvWebcam webcam;

    SequentialAction blueRightMiddle;

    @Override
    public void runOpMode() {
        BetterGamepad betterGamepad2 = new BetterGamepad(gamepad2);
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        propPipelineRedRight = new PropPipelineRedRight();
        robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueRight);

        autoConstants = new AutoConstants();

        initCamera();
        webcam.setPipeline(propPipelineRedRight);


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

        int tempHeight = 1500;


        SequentialAction transferAction = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.25),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)

        );
        SequentialAction openIntakeWhitePixelAction = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.TOP_43),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(1),
                intakeActions.openExtension(1100)
        );

        SequentialAction closeIntakeWhitePixelAction = new SequentialAction(
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(.5),
                intakeActions.moveStack(),
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                intakeActions.closeExtension()
        );
        SequentialAction intake5Action = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),

                new SleepAction(1),

                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                intakeActions.moveStack(),
                new SleepAction(.5),

                intakeActions.moveIntake(Intake.Angle.OUTTAKE),

                new SleepAction(1.5),
                intakeActions.moveClaw(Claw.ClawState.INTAKE, ClawSide.BOTH),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.5),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)

        );


        SequentialAction intake43Action = new SequentialAction(
                new SleepAction(1.5),

                intakeActions.moveIntake(Intake.Angle.TOP_54),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(1),
                intakeActions.openExtension(1000),

                new SleepAction(.75),

                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                intakeActions.moveStack(),

                new SleepAction(.5),
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                intakeActions.openExtension(-30),
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.75),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)

        );


        SequentialAction readyForDepositAction = new SequentialAction(
                new SleepAction(1),
                intakeActions.moveIntake(Intake.Angle.MID),
                new SleepAction(1),
                depositActions.readyForDeposit(tempHeight)

        );

        SequentialAction depositAction = new SequentialAction(

                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 0),
                new SleepAction(1),
                depositActions.moveElevator(1400),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),

                new SleepAction(1),
                depositActions.moveElevator(tempHeight),
                depositActions.retractDeposit()
        );

        SequentialAction fixIntake = new SequentialAction(

                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(0.1),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(0.2),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
                new SleepAction(0.3),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new InstantAction(() -> intakeExtension.setAggresive(false)),
                intakeActions.openExtension(75),
                new SleepAction(0.2),
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                intakeActions.openExtension(-20),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
                new SleepAction(0.2),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(0.2),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)

        );

        SequentialAction retractDeposit = new SequentialAction(
                depositActions.retractDeposit()
        );


        SequentialAction deposit43Action = new SequentialAction(
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),

                new SleepAction(0.5),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),

                new SleepAction(0.4),
                depositActions.moveElevator(tempHeight),
                depositActions.retractDeposit());

        //Trajectories

        Action placePurpleTraj = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-32, 33), Math.toRadians(-90))
                .build();

        Action intake5Traj = robot.drive.actionBuilder(new Pose2d(-34.5, 35.5, Math.toRadians(-90)))
                .setTangent(-180)
                .splineToSplineHeading(new Pose2d(-48, 41, Math.toRadians(0)), Math.toRadians(-90))
                .afterTime(.5, intake5Action)
                .splineToLinearHeading(new Pose2d(-46, 23, Math.toRadians(0)), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-52, 23.25), Math.toRadians(0))

                .strafeToLinearHeading(new Vector2d(-48, 23.25), Math.toRadians(0))
                .afterTime(.5, fixIntake)
                .strafeToLinearHeading(new Vector2d(-44.25, 10), Math.toRadians(0))

                .build();

        Action depositPreloadTraj = robot.drive.actionBuilder(new Pose2d(-44.25, 10, Math.toRadians(0)))

                //deposit
                .afterTime(0.25, readyForDepositAction)

                .strafeToLinearHeading(new Vector2d(30, 8), Math.toRadians(0))

                .splineToLinearHeading(new Pose2d(54, 28.5, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .afterTime(0, depositAction)

                .strafeToLinearHeading(new Vector2d(54, 33), Math.toRadians(0))

                //  .strafeToLinearHeading(new Vector2d(54, 36), Math.toRadians(0)).setTangent(0)

                .build();

        Action intake43Traj = robot.drive.actionBuilder(new Pose2d(54, 32, Math.toRadians(0)))
                .setTangent(Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(30, 9.5), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-30.5, 12, Math.toRadians(0)), Math.toRadians(180))
                .waitSeconds(.25)
                .afterTime(.5, fixIntake)

                .build();


        Action deposit43Traj = robot.drive.actionBuilder(new Pose2d(-30, 12, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(30, 12), Math.toRadians(0))
                .afterTime(.25, readyForDepositAction)
                .splineToLinearHeading(new Pose2d(52.5, 28, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .afterTime(0, deposit43Action)
                .build();

        Action parkTraj = robot.drive.actionBuilder(new Pose2d(52.25, 40, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(46, 32), Math.toRadians(-90))
                .build();

        ParallelAction placePurplePixel = new ParallelAction(
                placePurpleTraj
        );


        SequentialAction intake5 = new SequentialAction(
                intake5Traj
                //  intake5Action
        );

        ParallelAction depositPreload = new ParallelAction(
                depositPreloadTraj
        );

        ParallelAction intake43 = new ParallelAction(
                intake43Traj,
                intake43Action
        );
        SequentialAction park = new SequentialAction(
                parkTraj,
                depositActions.moveElevator(300)
        );

        blueRightMiddle = new SequentialAction(
                placePurplePixel,
                intake5,
                depositPreload,
                intake43,
                deposit43Traj
        );


        while (opModeInInit() && !isStopRequested()) {

            betterGamepad2.update();

            intake.setAngle(Intake.Angle.MID);

            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addData("POS", propPipelineRedRight.getLocation());
            telemetry.addData("NO PROP", propPipelineRedRight.NO_PROP);
            switch (propPipelineRedRight.getLocation()) {
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

            if (betterGamepad2.dpadUpOnce()) {
                propPipelineRedRight.NO_PROP++;
            } else if (betterGamepad2.dpadDownOnce()) {
                propPipelineRedRight.NO_PROP--;
            }
            telemetry.addLine("Initialized");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        switch (propLocation) {
            case LEFT:
                runBlocking(new ParallelAction(
                        blueRightMiddle,
                        updateActions.updateSystems()
                ));
                break;
            case RIGHT:
                runBlocking(new ParallelAction(
                        blueRightMiddle,
                        updateActions.updateSystems()
                ));
                break;
            case MIDDLE:
                runBlocking(new ParallelAction(
                        blueRightMiddle,
                        updateActions.updateSystems()
                ));
                break;
        }


        while (opModeIsActive()) {
            robot.drive.updatePoseEstimate();
        }

        writeToFile(robot.drive.pose.heading.log());
    }

    void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }

}