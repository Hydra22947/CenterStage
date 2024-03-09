package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import static org.firstinspires.ftc.teamcode.auto.AutoSettings.writeToFile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
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
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineBlueRight;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

@Config
@Autonomous(name = "2+1 - Auto Blue Right")
public class AutoRightBlue extends LinearOpMode {
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

    public enum PropLocation {
        LEFT,
        MIDDLE,
        RIGHT
    }


    public static PropLocation propLocation = PropLocation.RIGHT;
    PropPipelineBlueRight propPipelineBlueRight;
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        propPipelineBlueRight = new PropPipelineBlueRight();
        robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueRight);

        autoConstants = new AutoConstants();

        initCamera();
        webcam.setPipeline(propPipelineBlueRight);

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




        SequentialAction readyIntakeBlue = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.MID)
        );


        SequentialAction depositBlueMiddle = new SequentialAction(

                placePurpleActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(1),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),
                new SleepAction(0.5),
                depositActions.moveElevator(1400)
        );

        SequentialAction transferBlueMiddle = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                placePurpleActions.moveClaw(Claw.ClawState.OPEN, ClawSide.LEFT),
                placePurpleActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.75),
                placePurpleActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );


        SequentialAction retractDepositBlueMiddle = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction intakePixelBlueMiddle = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                placePurpleActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.LEFT),
                new SleepAction(1),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );



        SequentialAction intakePixelBlueRight = new SequentialAction(
                placePurpleActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                placePurpleActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                new SleepAction(1),
                placePurpleActions.openExtension(600)
        );

        SequentialAction intakePixelBlueClose = new SequentialAction(
                new SleepAction(1),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(0.5),
                placePurpleActions.moveStack(),
                placePurpleActions.closeExtension()

        );


        Action trajBlueLeft =
                robot.drive.actionBuilder(robot.drive.pose)
                        //place purple
                        .strafeToSplineHeading(new Vector2d(-37.5, 37.5), Math.toRadians(-40))
                        .splineToLinearHeading(new Pose2d(-34, 32, Math.toRadians(0)), Math.toRadians(0))


                        //intake from mid stack
                        .strafeToLinearHeading(new Vector2d(-53.75, 27.1), Math.toRadians(0))
                        .stopAndAdd(intakePixelBlueMiddle)


                        .waitSeconds(.5)
                        .stopAndAdd(transferBlueMiddle)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-35, 11), Math.toRadians(0))

                        .waitSeconds(8)

                        //deposit
                        .strafeToLinearHeading(new Vector2d(30, 9), Math.toRadians(0))
                        .afterDisp(.7,readyIntakeBlue)
                        .afterDisp(0.5, depositActions.readyForDeposit(1150))
                        //for no pixels change to 950

                        .splineToLinearHeading(new Pose2d(52, 40, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(depositBlueMiddle)
                        .waitSeconds(.5)

                        .strafeTo(new Vector2d(46, 39.5))
                        .stopAndAdd(retractDepositBlueMiddle)
                        .setTangent(Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(52, 10), Math.toRadians(-90))


                        //Park - Corner
                        //.lineToY(64)
                        .build();

        Action trajBlueMiddle =
                robot.drive.actionBuilder(robot.drive.pose)
                        //place purple
                        .strafeToLinearHeading(new Vector2d(-34.5, 34), Math.toRadians(-90))

                        //intake from mid stack
                        .strafeToLinearHeading(new Vector2d(-53.3, 27), Math.toRadians(0))
                        .stopAndAdd(intakePixelBlueMiddle)


                        .waitSeconds(.5)
                        .stopAndAdd(transferBlueMiddle)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-44.25, 7), Math.toRadians(0))

                        .waitSeconds(8)

                        //deposit
                        .strafeToLinearHeading(new Vector2d(30, 5), Math.toRadians(0))
                        .afterDisp(.7,readyIntakeBlue)
                        .afterDisp(0.5, depositActions.readyForDeposit(1100))
                        //for no pixels change to 950
                        .splineToLinearHeading(new Pose2d(52, 32.5, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(depositBlueMiddle)
                        .waitSeconds(.5)

                        //Park - Close to other board
                        .strafeTo(new Vector2d(46, 32))
                        .stopAndAdd(retractDepositBlueMiddle)
                        .setTangent(Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(50, 10), Math.toRadians(-90))




                        //Park - Corner
                        //.lineToY(64)
/*
                        //place purple
                        .strafeToLinearHeading(new Vector2d(-38, 34), Math.toRadians(-90))

                        //intake from mid stack
                        .setTangent(Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(-53.5, 24.5), Math.toRadians(0))
                        .stopAndAdd(intakePixelBlueMiddle)


                        .waitSeconds(.5)
                        .stopAndAdd(transferBlueMiddle)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-38, 10), Math.toRadians(0))

                        .waitSeconds(5)

                        //deposit
                        .strafeToLinearHeading(new Vector2d(30, 6), Math.toRadians(0))
                        .afterDisp(.7,readyIntakeBlue)
                        .afterDisp(0.9, depositActions.readyForDeposit(1000))
                        .splineToLinearHeading(new Pose2d(51.35, 30, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(depositBlueMiddle)
                        .waitSeconds(.5)

                        .setTangent(Math.toRadians(90))
                        .stopAndAdd(retractDepositBlueMiddle)
                        //Park - Close to other board
                        .strafeToLinearHeading(new Vector2d(55, 10), Math.toRadians(0))
                        .turnTo(Math.toRadians(-90))

                        //Park - Corner
                        //.lineToY(64)*/
                        .build();
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50),
                new AngularVelConstraint(Math.toRadians(150))));

        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10, 25);

        Action trajBlueRight =
                robot.drive.actionBuilder(robot.drive.pose)
                        //place purple
                        .strafeToLinearHeading(new Vector2d(-49.5, 44), Math.toRadians(-90))


                        //intake from left stack
                        .strafeToSplineHeading(new Vector2d(-42, 45), Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(-34, 15, Math.toRadians(-90)), Math.toRadians(-90))
                        .strafeToLinearHeading(new Vector2d(-26, 14.15), Math.toRadians(0)
                        , baseVelConstraint,baseAccelConstraint)
                        .stopAndAdd(intakePixelBlueRight)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-39.25, 14.5), Math.toRadians(0))
                        .waitSeconds(.5)
                        .stopAndAdd(intakePixelBlueClose)
                        .waitSeconds(.5)
                        .stopAndAdd(transferBlueMiddle)

                        .waitSeconds(8)

                        //deposit
                        .strafeToLinearHeading(new Vector2d(30, 9), Math.toRadians(0))
                        .afterDisp(.7,readyIntakeBlue)
                        .afterDisp(0.5, depositActions.readyForDeposit(1100))
                        //for no pixels change to 950
                        .splineToLinearHeading(new Pose2d(52, 28.25, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(depositBlueMiddle)
                        .waitSeconds(.5)
                        .lineToX(48)

                        .strafeTo(new Vector2d(46, 28))
                        .stopAndAdd(retractDepositBlueMiddle)
                        .setTangent(Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(50, 10), Math.toRadians(-90))

                        //Park - Corner
                        //.lineToY(64)
                        .build();

        while (opModeInInit() && !isStopRequested()) {
            intake.setAngle(Intake.Angle.MID);
            intakeExtension.setTarget(0);
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addData("POS", propPipelineBlueRight.getLocation());
             switch (propPipelineBlueRight.getLocation()) {
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
        robot.drive.startAutoTimer();

        switch (propLocation) {
            case LEFT:
                runBlocking(new ParallelAction(
                        trajBlueLeft,
                        updateActions.updateSystems()
                ));
                break;
            case MIDDLE:
                runBlocking(new ParallelAction(
                        trajBlueMiddle,
                        updateActions.updateSystems()
                ));
                break;
            case RIGHT:
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