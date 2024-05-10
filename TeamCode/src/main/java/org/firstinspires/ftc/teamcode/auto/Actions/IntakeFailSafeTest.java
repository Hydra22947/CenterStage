package org.firstinspires.ftc.teamcode.auto.Actions;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineRedLeft;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "intakeFailSafeTest")
@Disabled
public class IntakeFailSafeTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    ElapsedTime time;

    // subsystems
    LiftSubsystem elevator;
    IntakeSubsystem intake;
    OuttakeSubsystem outtake;
    Claw claw;
    IntakeExtensionSubsystem intakeExtension;
    AutoConstants autoConstants;

    DepositActions depositActions;
    PlacePurpleActions placePurpleActions;
    UpdateActions updateActions;

    public static double fixPoseIntakeY = 0;

    enum PropLocation {
        LEFT,
        MIDDLE,
        RIGHT
    }

    PropLocation propLocation = PropLocation.MIDDLE;
    PropPipelineRedLeft propPipelineRedLeft;
    OpenCvWebcam webcam;

    enum IntakeFailSafe {

        ACTIVATED,
        DEACTIVATED

    }


    IntakeFailSafe intakeFailSafe;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        propPipelineRedLeft = new PropPipelineRedLeft();
        robot.init(hardwareMap, telemetry, autoConstants.startPoseRedLeft);

        autoConstants = new AutoConstants();

        initCamera();
        webcam.setPipeline(propPipelineRedLeft);

        time = new ElapsedTime();

        robot.init(hardwareMap, telemetry, autoConstants.startPoseRedLeft);

        autoConstants = new AutoConstants();

        elevator = new LiftSubsystem(true);
        outtake = new OuttakeSubsystem();
        claw = new Claw();
        intake = new IntakeSubsystem();
        intakeExtension = new IntakeExtensionSubsystem(true);

        intakeExtension.setAuto(true);
        elevator.setAuto(true);

        depositActions = new DepositActions(elevator, intake, claw, outtake, intakeExtension);
        placePurpleActions = new PlacePurpleActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);

        intakeFailSafe = IntakeFailSafe.ACTIVATED;

        SequentialAction depositRedMiddle = new SequentialAction(
                placePurpleActions.moveIntake(IntakeSubsystem.Angle.MID),
                depositActions.readyForDeposit(1100),
                placePurpleActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(0.5),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 600)
        );
        SequentialAction transferRedMiddle = new SequentialAction(
                placePurpleActions.moveIntake(IntakeSubsystem.Angle.OUTTAKE),
                new SleepAction(0.5),
                placePurpleActions.moveClaw(Claw.ClawState.OPEN, ClawSide.RIGHT),
                placePurpleActions.moveIntakeClaw(IntakeSubsystem.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(.5),
                placePurpleActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );
        SequentialAction placePurplePixelCloseRedMiddle = new SequentialAction(

                new SleepAction(1.5),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(1),
                placePurpleActions.moveIntake(IntakeSubsystem.Angle.MID),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction retractDepositRedMiddle = new SequentialAction(
                depositActions.retractDeposit()
        );


        SequentialAction intakePixelRedMiddle = new SequentialAction(
                placePurpleActions.moveIntakeClaw(IntakeSubsystem.ClawState.OPEN, ClawSide.BOTH),
                placePurpleActions.moveIntake(IntakeSubsystem.Angle.TOP_5_AUTO),
                new SleepAction(.7),
                placePurpleActions.openExtension(750),
                new SleepAction(1),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(0.5),
                placePurpleActions.moveStack(),
                placePurpleActions.closeExtension()

        );
        SequentialAction readyIntakeRedMiddle = new SequentialAction(
                placePurpleActions.moveIntake(IntakeSubsystem.Angle.INTAKE)
        );


        Action trajRedMiddle =
                robot.drive.actionBuilder(robot.drive.pose)

                        .setTangent(Math.toRadians(-70))
                        .lineToY(-12)
                        .stopAndAdd(readyIntakeRedMiddle)
                        .setTangent(0)
                        .strafeTo(new Vector2d(-33, -10.5))
                        .stopAndAdd(placePurplePixelCloseRedMiddle)
                        .waitSeconds(.2)


                        //Park - Corner
                        //.lineToY(64)
                        .build();


        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 25.0);

        Action goForIntake = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-38.5, -11.5), Math.toRadians(0))
                .waitSeconds(.5)
                .stopAndAdd(intakePixelRedMiddle)
                .waitSeconds(5)
                .stopAndAdd(transferRedMiddle)
                .stopAndAdd(readyIntakeRedMiddle)

                .build();

        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() >= -11.5) {
                return 20.0;
            } else {
                return 50.0;
            }
        };

        Action goForPlacement = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(30, -9), Math.toRadians(0))
                .afterDisp(0.9, depositActions.readyForDeposit(1100))
                .afterDisp(1, placePurpleActions.moveIntake(IntakeSubsystem.Angle.MID))
                .splineToLinearHeading(new Pose2d(51.55, -32.5, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .stopAndAdd(depositRedMiddle)
                .waitSeconds(.5)
                .setTangent(Math.toRadians(90))
                .stopAndAdd(retractDepositRedMiddle)
                //Park - Close to other board
                .lineToY(-10)
                .turnTo(Math.toRadians(90))
                .build();


        while (opModeInInit() && !isStopRequested()) {
            intake.setAngle(IntakeSubsystem.Angle.MID);
            intake.updateState(IntakeSubsystem.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            robot.drive.updatePoseEstimate();
            switch (propPipelineRedLeft.getLocation()) {
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
        webcam.stopStreaming();

        if (isStopRequested()) return;

        switch (propLocation) {
            case RIGHT:
                runBlocking(new ParallelAction(
                        //  trajRedRight,
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
                        //trajRedLeft,
                        updateActions.updateSystems()
                ));
                break;
        }

        while (opModeIsActive()) {
            robot.drive.updatePoseEstimate();
        }


        runBlocking(trajRedMiddle);
        runBlocking(goForIntake);

        switch (intakeFailSafe) {

            case ACTIVATED:

                if (robot.drive.pose.position.y == 11) {
                    fixPoseIntakeY++;
                }

                if (robot.colorLeft.getDistance(DistanceUnit.INCH) <= 10 && robot.colorRight.getDistance(DistanceUnit.INCH) <= 10) {
                    intakeFailSafe = IntakeFailSafe.DEACTIVATED;
                }

                break;
            case DEACTIVATED:

                runBlocking(goForPlacement);

                break;

            default:

                intakeFailSafe = IntakeFailSafe.ACTIVATED;
        }

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

