package org.firstinspires.ftc.teamcode.auto.MaxAuto.AutoRedRight;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoSettings.writeToFile;

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
@Autonomous(name = "2+2 - Auto Right Red MAX")
public class AutoRightRedMax extends LinearOpMode {
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

    public static int tempHeight = 900;

    @Override
    public void runOpMode() {
        BetterGamepad betterGamepad2 = new BetterGamepad(gamepad2);
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        propPipelineRedRight = new PropPipelineRedRight();
        robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueLeft);

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

        SequentialAction depositBlue = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),
                new SleepAction(0.5),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 0),
                new SleepAction(.2),
                depositActions.moveElevator(1300),
                new SleepAction(.2),
                intakeActions.moveIntake(Intake.Angle.TOP_54_AUTO),
                depositActions.retractDeposit()

        );
        SequentialAction readyForDepositAction = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),
                depositActions.readyForDeposit(tempHeight)

        );

        SequentialAction depositSecondCycle = new SequentialAction(
                new SleepAction(3),
                readyForDepositAction,

                intakeActions.moveIntake(Intake.Angle.MID),

                intakeActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(1),
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),

                new SleepAction(0.1),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),

                new SleepAction(0.4),
                depositActions.moveElevator(tempHeight),
                depositActions.retractDeposit()
        );

        SequentialAction placePurplePixelAction = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.INTAKE),
                intakeActions.openExtension(600),

                new SleepAction(0.25),
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN)

        );

        SequentialAction retractPurpleAction = new SequentialAction(
                new SleepAction(0.3),
                intakeActions.closeExtension(),
                intakeActions.moveIntake(Intake.Angle.MID),
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction openIntakeWhitePixelAction = new SequentialAction(
                new SleepAction(1.5),
                intakeActions.moveIntake(Intake.Angle.TOP_54),
                new SleepAction(.5),
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.5),
                intakeActions.openExtension(1000)

        );

        SequentialAction closeIntakeWhitePixelAction = new SequentialAction(
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(.5),
                intakeActions.moveStack(),
                new SleepAction(.5),
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                intakeActions.closeExtension()
        );


        SequentialAction transferAction = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.75),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );


        SequentialAction retractDepositBlueMaxAction = new SequentialAction(
                depositActions.retractDeposit()
        );


        SequentialAction placePurplePixelSequence = new SequentialAction(
                depositActions.readyForDeposit(950),
                new SleepAction(1.5),
                   placePurplePixelAction,
                   retractPurpleAction

        );

        SequentialAction intake54Action = new SequentialAction(
                openIntakeWhitePixelAction,
                new SleepAction(1.5),
                closeIntakeWhitePixelAction
        );
        //Trajectories
        Action placePurpleTraj = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(48, -38), Math.toRadians(0))
                .waitSeconds(1.5)
                .build();

        Action placeYellowPixelTraj = robot.drive.actionBuilder(new Pose2d(48, -34., Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(50.75, -34, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1)
                .build();

        Action intake54Traj = robot.drive.actionBuilder(new Pose2d(50.25, -34, Math.toRadians(0)))

                //Going to intake position
                .setTangent(Math.toRadians(-120))
                .splineToConstantHeading(new Vector2d(30, -9.5), Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(-28, -10.84, Math.toRadians(0)), Math.toRadians(180))

                //Getting Closer and fixing angle
                .strafeToLinearHeading(new Vector2d(-33.25, -10.5), Math.toRadians(0))
                .waitSeconds(.25)
                .build();

        Action place54Traj = robot.drive.actionBuilder(new Pose2d(-33, -10.84, Math.toRadians(0)))

                .strafeToLinearHeading(new Vector2d(30, -9), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51, -28, Math.toRadians(-5)), Math.toRadians(0))
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
                intake54Action
        );

        ParallelAction deposit54 = new ParallelAction(
                place54Traj,
                depositSecondCycle
        );


        SequentialAction rightRedRight= new SequentialAction(
                placePurplePixel,
                placePreloadOnBoard,
                intake54,
                deposit54
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
                        rightRedRight,
                        updateActions.updateSystems()
                ));
                break;
            case RIGHT:
                runBlocking(new ParallelAction(
                        rightRedRight,
                        updateActions.updateSystems()
                ));
                break;
            case MIDDLE:
                runBlocking(new ParallelAction(
                        rightRedRight,
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