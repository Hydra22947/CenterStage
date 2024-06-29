package org.firstinspires.ftc.teamcode.auto.OldAuto.BasicAutos;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.Actions.IntakeActions;
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
@Autonomous(name = "2+1 - Red Left")
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
    IntakeActions intakeActions;
    UpdateActions updateActions;

    public static AutoSettings.PropLocation propLocation = AutoSettings.PropLocation.RIGHT;
    PropPipelineRedLeft propPipelineRedLeft;
    OpenCvWebcam webcam;
    boolean first = true;
    int elevatorHeightMin = 950;
    int elevatorHeightMax = 1200;

    int elevatorHeight = elevatorHeightMax;
    boolean vision = true;

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
        intakeActions = new IntakeActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);


        SequentialAction depositBlueMiddle = new SequentialAction(
                depositActions.placePixel()
        );

        SequentialAction depositIntermediate = new SequentialAction(

                //intakeActions.failSafeClaw(IntakeActions.FailSafe.ACTIVATED),
                new SleepAction(1),
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 0),

                new SleepAction(0.2)
        );

        SequentialAction depositTwoPixels = new SequentialAction(

                // intakeActions.failSafeClaw(IntakeActions.FailSafe.ACTIVATED),
                new SleepAction(1),
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 0),

                new SleepAction(0.2),
                //depositActions.placePixel(),

                new SleepAction(.25),
                depositActions.moveElevator(1850)
        );
        SequentialAction transferBlueMiddle = new SequentialAction(
                intakeActions.openExtension(-50),
                depositActions.moveOuttake(Outtake.Angle.ALMOST_INTAKE),
                depositActions.moveClaw(Claw.ClawState.INTAKE, ClawSide.BOTH),
                new SleepAction(.25),
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(.75),
                depositActions.moveOuttake(Outtake.Angle.INTAKE),
                new SleepAction(.5),
                depositActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
                new SleepAction(.2),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.5),
                intakeActions.moveIntake(Intake.Angle.TELEOP_MID)

        );


        SequentialAction retractDepositBlueMiddle = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction intakePixelBlueMiddle = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(1),
                intakeActions.lock(IntakeActions.CloseClaw.BOTH_CLOSE)

        );

        SequentialAction intakePixelBlueLeft = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.RIGHT)
        );

        SequentialAction intakePixelBlueRight = new SequentialAction(
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.RIGHT),
                intakeActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                new SleepAction(1),
                intakeActions.openExtension(700)
        );

        SequentialAction intakePixelBlueClose = new SequentialAction(
                intakeActions.lock(IntakeActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(0.5),
                intakeActions.moveStack(),
                intakeActions.openExtension(-20)

        );
        SequentialAction readyForDeposit = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.TELEOP_MID),
                new SleepAction(0.5),
                depositActions.readyForDeposit(elevatorHeight),
                new InstantAction(() -> outtake.spinOuttake(1))
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
                        .stopAndAdd(intakeActions.lock(IntakeActions.CloseClaw.BOTH_CLOSE))

                        .waitSeconds(.5)
                        .stopAndAdd(transferBlueMiddle)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-35, -11), Math.toRadians(0))

                        //deposit
                        .strafeToLinearHeading(new Vector2d(30, -12), Math.toRadians(0))
                        .afterDisp(15, readyForDeposit)
                        //for no pixels change to 950

                        .splineToLinearHeading(new Pose2d(48, -42, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(depositBlueMiddle)
                        .strafeToLinearHeading(new Vector2d(46, -42), Math.toRadians(0))
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
                        .strafeToLinearHeading(new Vector2d(-48, -26), Math.toRadians(0))
                        .stopAndAdd(intakePixelBlueLeft)
                        .waitSeconds(.1)
                        .strafeToLinearHeading(new Vector2d(-53.5, -26), Math.toRadians(0))
                        .waitSeconds(.1)
                        .stopAndAdd(intakeActions.lock(IntakeActions.CloseClaw.BOTH_CLOSE))

                        .waitSeconds(.5)
                        .stopAndAdd(transferBlueMiddle)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-44.25, -10), Math.toRadians(0))

                        //deposit
                        .strafeToLinearHeading(new Vector2d(30, -8.5), Math.toRadians(0))
                        .afterDisp(25, readyForDeposit)

                        .splineToLinearHeading(new Pose2d(48, -32.5, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(depositBlueMiddle)
                        .strafeToLinearHeading(new Vector2d(46, -32.5), Math.toRadians(0))
                        .waitSeconds(.5)
                        .setTangent(Math.toRadians(-90))
                        //Park - Close to other board
                        .strafeToLinearHeading(new Vector2d(46, -10), Math.toRadians(90))
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
                        .splineToLinearHeading(new Pose2d(-38, -14, Math.toRadians(90)), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-35, -14), Math.toRadians(0)
                                , baseVelConstraint, baseAccelConstraint)
                        .stopAndAdd(intakePixelBlueRight)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-38, -14), Math.toRadians(0))
                        .waitSeconds(.25)
                        .stopAndAdd(intakePixelBlueClose)
                        .waitSeconds(.5)
                        .stopAndAdd(transferBlueMiddle)

                        //deposit
                        .strafeToLinearHeading(new Vector2d(20, -9.5), Math.toRadians(0))
                        .afterDisp(0, readyForDeposit)

                        //for no pixels change to 950
                        .splineToLinearHeading(new Pose2d(50.5, -30, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(depositBlueMiddle)
                        .strafeToLinearHeading(new Vector2d(46, -30), Math.toRadians(0))
                        .waitSeconds(.5)
                        .setTangent(Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(46.5, -10), Math.toRadians(90))
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

            if(vision)
            {
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
            }

            if (gamepad1.dpad_right) {
                elevatorHeight = elevatorHeightMax;
            } else if (gamepad2.dpad_left) {
                elevatorHeight = elevatorHeightMin;
            }

            if(gamepad1.y)
            {
                vision = false;
                propLocation = AutoSettings.PropLocation.MIDDLE;
            }
            else if(gamepad1.b)
            {
                vision = false;
                propLocation = AutoSettings.PropLocation.RIGHT;
            }
            else if(gamepad1.x)
            {
                vision = false;
                propLocation = AutoSettings.PropLocation.LEFT;
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

    SequentialAction returnFixintake() {
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