package org.firstinspires.ftc.teamcode.auto.BasicAutos;

// RR-specific imports

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import static org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoSettings.cycleVision;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoSettings;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineRedLeft;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

@Config
@Autonomous(name = "2+1 - Auto Red Left")
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

    public static AutoSettings.PropLocation propLocation = AutoSettings.PropLocation.RIGHT;
    PropPipelineRedLeft propPipelineRedLeft;
    OpenCvWebcam webcam;
    boolean first = true;
    int elevatorHeightMin = 950;
    int elevatorHeightMax = 1150;

    int elevatorHeight = elevatorHeightMax;

    @Override
    public void runOpMode() {
        first = true;
        BetterGamepad betterGamepad2 = new BetterGamepad(gamepad2);
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


        SequentialAction readyIntakeBlue = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.MID)
        );


        SequentialAction depositBlueMiddle = new SequentialAction(

                placePurpleActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(1),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),
                new SleepAction(0.5),
                depositActions.moveElevator(elevatorHeight + 300)
        );
        SequentialAction depositIntermediate = new SequentialAction(

                placePurpleActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(1),
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 0),

                new SleepAction(0.2)
        );

        SequentialAction depositTwoPixels = new SequentialAction(

                placePurpleActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(1),
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 0),

                new SleepAction(0.2),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),

                new SleepAction(.25),
                depositActions.moveElevator(1850)
        );
        SequentialAction transferBlueMiddle = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                placePurpleActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                placePurpleActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(1),
                placePurpleActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );


        SequentialAction retractDepositBlueMiddle = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction intakePixelBlueMiddle = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                placePurpleActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(.5),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)

        );

        SequentialAction intakePixelBlueLeft = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                placePurpleActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH)

        );

        SequentialAction intakePixelBlueRight = new SequentialAction(
                placePurpleActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                placePurpleActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                new SleepAction(1),
                placePurpleActions.openExtension(700)
        );

        SequentialAction intakePixelBlueClose = new SequentialAction(
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(0.5),
                placePurpleActions.moveStack(),
                placePurpleActions.openExtension(-35)

        );
      /*  SequentialAction readyForDeposit = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.MID),
                new SleepAction(.25),
                depositActions.readyForDeposit(1300)
        );*/
        SequentialAction readyForDeposit = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.MID),
                new SleepAction(.25),
                depositActions.readyForDeposit(elevatorHeight)
        );

        Action trajRedRight =
                robot.drive.actionBuilder(robot.drive.pose)
                        //place purple
                        .strafeToSplineHeading(new Vector2d(-37.5, -37.5), Math.toRadians(40))
                        .splineToLinearHeading(new Pose2d(-34, -32, Math.toRadians(0)), Math.toRadians(0))
                        //32

                        //intake from mid stack
                        .stopAndAdd(intakePixelBlueLeft)
                        .strafeToLinearHeading(new Vector2d(-53.5, -25), Math.toRadians(0))
                        .waitSeconds(.1)
                        .stopAndAdd(placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE))

                        .waitSeconds(.5)
                        .stopAndAdd(transferBlueMiddle)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-35, -11), Math.toRadians(0))

                        .stopAndAdd(new ParallelAction(new SleepAction(9.5), returnFixintake()))

                        //deposit
                        .strafeToLinearHeading(new Vector2d(30, -12), Math.toRadians(0))
                        .afterDisp(.7, readyIntakeBlue)
                        .afterDisp(0.5, readyForDeposit)
                        //for no pixels change to 950

                        .splineToLinearHeading(new Pose2d(52.25, -38, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(depositIntermediate)
                        .splineToLinearHeading(new Pose2d(52.25, -42, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(depositBlueMiddle)
                        .waitSeconds(.5)
                        .setTangent(Math.toRadians(-90))

                        .strafeToLinearHeading(new Vector2d(46, -39.5), Math.toRadians(90))
                        .stopAndAdd(retractDepositBlueMiddle)


                        //Park - Corner
                        //.lineToY(64)
                        .build();

        Action trajRedMiddle =
                robot.drive.actionBuilder(robot.drive.pose)
                        //place purple
                        .strafeToLinearHeading(new Vector2d(-28, -34), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-34.5, -42), Math.toRadians(90))

                        //intake from mid stack
                        .strafeToLinearHeading(new Vector2d(-48, -22), Math.toRadians(0))
                        .stopAndAdd(intakePixelBlueLeft)
                        .waitSeconds(.1)
                        .strafeToLinearHeading(new Vector2d(-53.5, -21.1), Math.toRadians(0))
                        .waitSeconds(.1)
                        .stopAndAdd(placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE))


                        .waitSeconds(.5)
                        .stopAndAdd(transferBlueMiddle)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-44.25, -10), Math.toRadians(0))

                        .stopAndAdd(new ParallelAction(new SleepAction(10), returnFixintake()))

                        //deposit
                        .strafeToLinearHeading(new Vector2d(30, -8.5), Math.toRadians(0))
                        .afterDisp(.7, readyIntakeBlue)
                        //for no pixels change to 950
                        .splineToLinearHeading(new Pose2d(52, -35.75, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(depositBlueMiddle)
                        .waitSeconds(.5)
                        .setTangent(Math.toRadians(-90))
                        //Park - Close to other board
                        .strafeToLinearHeading(new Vector2d(46, -32), Math.toRadians(90))
                        .stopAndAdd(retractDepositBlueMiddle)
                        .build();

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50),
                new AngularVelConstraint(Math.toRadians(150))));

        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10, 25);

        Action trajRedLeft =
                robot.drive.actionBuilder(robot.drive.pose)
                        //place purple
                        .strafeToLinearHeading(new Vector2d(-50, -44), Math.toRadians(90))


                        //intake from left stack
                        .strafeToSplineHeading(new Vector2d(-42, -48), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(-38, -12, Math.toRadians(90)), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-29.9, -10), Math.toRadians(0)
                                , baseVelConstraint, baseAccelConstraint)
                        .stopAndAdd(intakePixelBlueRight)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-37.25, -11), Math.toRadians(0))
                        .waitSeconds(.25)
                        .stopAndAdd(intakePixelBlueClose)
                        .waitSeconds(.5)
                        .stopAndAdd(transferBlueMiddle)

                        .stopAndAdd(new ParallelAction(new SleepAction(8.5), returnFixintake()))

                        //deposit
                        .strafeToLinearHeading(new Vector2d(30.25, -9.5), Math.toRadians(0))
                        .afterDisp(.7, readyIntakeBlue)
                        //for no pixels change to 950
                        .splineToLinearHeading(new Pose2d(52.75, -29, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(depositBlueMiddle)
                        .waitSeconds(.5)
                        .lineToX(48)
                        .setTangent(Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(46.5, -28), Math.toRadians(90))
                        .stopAndAdd(retractDepositBlueMiddle)

                        //Park - Corner
                        //.lineToY(64)
                        .build();

        while (opModeInInit() && !isStopRequested()) {
            betterGamepad2.update();
            intake.setAngle(Intake.Angle.MID);
            intakeExtension.setTarget(0);
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addData("POS", propLocation.name());
            telemetry.addData("elevator pos", elevatorHeight);

           switch (propPipelineRedLeft.getLocation()) {
                case Left:
                    propLocation = AutoSettings.PropLocation.LEFT;
                    break;
                case Right:
                    propLocation = AutoSettings.PropLocation.RIGHT;
                    break;
                case Center:
                    propLocation = AutoSettings.PropLocation.MIDDLE;
                    break;
            }

            if(betterGamepad2.dpadRightOnce())
            {
                elevatorHeight = elevatorHeightMax;
            }
            else if(betterGamepad2.dpadLeftOnce())
            {
                elevatorHeight = elevatorHeightMin;
            }

            if(betterGamepad2.dpadUpOnce())
            {
                if(first)
                {
                    webcam.stopStreaming();
                    first = false;
                }

                propLocation = cycleVision(propLocation);
            }
            else if(betterGamepad2.dpadDownOnce())
            {
                initCamera();
                webcam.setPipeline(propPipelineRedLeft);
            }

            telemetry.addLine("Initialized");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        webcam.stopStreaming();

        switch (propLocation) {
            case LEFT:
                runBlocking(new ParallelAction(
                        trajRedLeft,
                        updateActions.updateSystems()
                ));
                break;
            case MIDDLE:
                runBlocking(new ParallelAction(
                        trajRedMiddle,
                        updateActions.updateSystems()
                ));
                break;
            case RIGHT:
                runBlocking(new ParallelAction(
                        trajRedRight,
                        updateActions.updateSystems()
                ));
                break;
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

    SequentialAction returnFixintake () {
        return new SequentialAction(
                new SleepAction(.5),
                new InstantAction(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH)),
                new SleepAction(0.1),
                new InstantAction(() -> claw.setBothClaw(Claw.ClawState.OPEN)),
                new SleepAction(0.2),
                new InstantAction(() -> claw.setBothClaw(Claw.ClawState.CLOSED)),
                new SleepAction(0.3),
                new InstantAction(() -> claw.setBothClaw(Claw.ClawState.OPEN)),
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                new InstantAction(() -> intakeExtension.setTarget(250)),
                new SleepAction(0.2),
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                new InstantAction(() -> intakeExtension.setTarget(0)),
                new InstantAction(() -> claw.setBothClaw(Claw.ClawState.CLOSED)),
                new SleepAction(0.2),
                new InstantAction(() -> claw.setBothClaw(Claw.ClawState.OPEN)),
                new SleepAction(0.2),
                new InstantAction(() -> claw.setBothClaw(Claw.ClawState.CLOSED))

        );
    }


}