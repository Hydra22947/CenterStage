package org.firstinspires.ftc.teamcode.auto.Disabled;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoSettings.cycleVision;

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

@Config
@Autonomous(name = "2+1 - AutoBlueRight Left Red NEW")
@Disabled
public class AutoLeftRedNew extends LinearOpMode {
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
    boolean shouldUseAprilTag = true;

    public static AutoSettings.PropLocation propLocation = AutoSettings.PropLocation.MIDDLE;

    PropPipelineRedLeft propPipelineRedLeft;
    OpenCvWebcam webcam;

    SequentialAction blueRightLeft, blueRightMiddle, blueRightRight;
    SequentialAction readyForDepositAction;
    int tempHeight = 1250;
    int tempHeightMin = 1050;
    int tempHeightMax = 1250;

    boolean first = true;

    @Override
    public void runOpMode() {
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

        SequentialAction intake5OpenAction = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH)

        );

        SequentialAction intake5CloseAction = new SequentialAction(
                intakeActions.lock(IntakeActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(.5),
                intakeActions.moveStack(),

                intakeActions.moveIntake(Intake.Angle.OUTTAKE),

                new SleepAction(0.5),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.5),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)

        );


        SequentialAction intake43OpenAction = new SequentialAction(


                intakeActions.moveIntake(Intake.Angle.TOP_54_AUTO),
                new InstantAction(() -> intakeExtension.setAggresive(true)),

                new ParallelAction(

                        intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                        intakeActions.openExtension(450)
                )
        );

        SequentialAction intake43OpenActionRight = new SequentialAction(


                intakeActions.moveIntake(Intake.Angle.TOP_43),
                new InstantAction(() -> intakeExtension.setAggresive(true)),

                new ParallelAction(

                        intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                        intakeActions.openExtension(450)
                )
        );


        SequentialAction intake43CloseAction = new SequentialAction(
                new SleepAction(.2),
                intakeActions.lock(IntakeActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(.5),
                intakeActions.closeExtension(),
                new SleepAction(.5),
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),


                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(.5),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.25),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH)


        );

        SequentialAction depositAction = new SequentialAction(

                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 0),
                new SleepAction(1.4),
               // depositActions.placePixel(),
                new SleepAction(.5),
                depositActions.moveElevator(tempHeight + 400),
                depositActions.retractDeposit()
        );

        readyForDepositAction = new SequentialAction(
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
                intakeActions.moveIntake(Intake.Angle.TELEOP_MID),
                new SleepAction(.75),
                depositActions.readyForDeposit(tempHeight));


        SequentialAction deposit43Action = new SequentialAction(
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),
                new SleepAction(0.6),
               // depositActions.placePixel(),

                new SleepAction(0.25),
                depositActions.moveElevator(tempHeight + 300),
                depositActions.retractDeposit());

        //Trajectories

        Action placePurpleTrajRight = robot.drive.actionBuilder(robot.drive.pose)
                .splineToLinearHeading(new Pose2d(-29, -40, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Action placePurpleTrajMiddle = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-32, -32.5), Math.toRadians(90))
                .build();

        Action placePurpleTrajLeft = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-45, -43), Math.toRadians(90))
                .build();

        Action intake5TrajLeft = robot.drive.actionBuilder(new Pose2d(-48, -44, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(-42, -45), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-38, -11, Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-45, -11), Math.toRadians(0))
                .stopAndAdd(intake5OpenAction)
                .strafeToLinearHeading(new Vector2d(-53, -11), Math.toRadians(0))
                .stopAndAdd(intake5CloseAction)
                .build();

        Action intake5TrajMiddle = robot.drive.actionBuilder(new Pose2d(-32, -32.5, Math.toRadians(90)))
                .setTangent(180)
                .splineToSplineHeading(new Pose2d(-48, -41, Math.toRadians(0)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-48, -26, Math.toRadians(0)), Math.toRadians(-180))
                .afterTime(0, intake5OpenAction)
                .waitSeconds(0.25)
                .strafeToLinearHeading(new Vector2d(-53.1, -26), Math.toRadians(0))
                .stopAndAdd(intake5CloseAction)
                .build();

        Action intake5TrajRight = robot.drive.actionBuilder(new Pose2d(-50, -43, Math.toRadians(0)))
                .strafeToSplineHeading(new Vector2d(-42, -45), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-38, -14, Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-45, -14), Math.toRadians(0))
                .stopAndAdd(intake5OpenAction)
                .strafeToLinearHeading(new Vector2d(-53.1, -14), Math.toRadians(0))
                .stopAndAdd(intake5CloseAction)
                .build();

//        VelConstraint baseVelConstraint = new VelConstraint() {
//            @Override
//            public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
//                if (pose2dDual.position.x.value() > 30 && pose2dDual.position.x.value() < 37.5) {
//                    return 5;
//                } else {
//                    return 50.0;
//                }
//            }
//        };

        Action depositPreloadTrajLeft =// new CheckAprilTagAction(shouldUseAprilTag, time,
                robot.drive.actionBuilder(new Pose2d(-53, -11, Math.toRadians(0)))
                        .afterTime(0.5, pleaseFixIntakeClose())

                        .splineToLinearHeading(new Pose2d(-42, -10, Math.toRadians(0)), Math.toRadians(0))
                        //deposit
                        .afterTime(.5, readyForDepositAction)

                        .splineToLinearHeading(new Pose2d(30, -10, Math.toRadians(0)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(52, -36, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .afterTime(0, new InstantAction(() -> claw.setRightClaw(Claw.ClawState.OPEN)))
                        .afterTime(0, depositAction)
                        .strafeToLinearHeading(new Vector2d(52, -30), Math.toRadians(0))

                        .build();

        //,
//                new SequentialAction(robot.drive.actionBuilder(new Pose2d(31, 20, Math.toRadians(0)))
//                        .strafeToLinearHeading(new Vector2d(50.5, 30), Math.toRadians(-90)).build(),
//                        new InstantAction(() -> intake.move(Intake.Angle.TELEOP_MID)),
//                        new InstantAction(() -> claw.setBothClaw(Claw.ClawState.CLOSED)),
//                        new InstantAction(() -> outtake.setAngle(Outtake.Angle.INTAKE)),
//                        new InstantAction(() -> elevator.setTarget(0)),
//                        new SleepAction(1),
//                        new InstantAction(() -> requestOpModeStop()))
//        );

        Action depositPreloadTrajMiddle = robot.drive.actionBuilder(new Pose2d(-53, -26, Math.toRadians(0)))

                .afterTime(0, pleaseFixIntakeClose())

                .splineToLinearHeading(new Pose2d(-42, -10, Math.toRadians(0)), Math.toRadians(0))
                .afterTime(.5, readyForDepositAction)

                .splineToLinearHeading(new Pose2d(30, -10, Math.toRadians(0)), Math.toRadians(0))

                //deposit
                .splineToLinearHeading(new Pose2d(52, -28, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .afterTime(0, depositAction)
                .strafeToSplineHeading(new Vector2d(52, -38), Math.toRadians(0))
                .build();

        Action depositPreloadTrajRight = robot.drive.actionBuilder(new Pose2d(-53.25, -14, Math.toRadians(0)))
                .afterTime(0, pleaseFixIntakeClose())
                .afterTime(1, readyForDepositAction)

                //deposit
                .strafeToLinearHeading(new Vector2d(30, -12), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(52, -30, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .afterTime(0, depositAction)
                .strafeToSplineHeading(new Vector2d(50.6, -44), Math.toRadians(5))
                .build();


        Action intake43Traj_Left = robot.drive.actionBuilder(new Pose2d(50, -30, Math.toRadians(0)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24, -9.5, Math.toRadians(0)), Math.toRadians(-180))
                .afterTime(0.9, intake43OpenAction)
                .splineToLinearHeading(new Pose2d(-35.9, -9.5, Math.toRadians(0)), Math.toRadians(180))
                .waitSeconds(.2)
                .afterTime(0.6, intake43CloseAction)
                .strafeToLinearHeading(new Vector2d(-44, -10.8), Math.toRadians(0))
                .build();


        Action intake43Traj_Middle = robot.drive.actionBuilder(new Pose2d(51, -38, Math.toRadians(0)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24, -9.5, Math.toRadians(0)), Math.toRadians(-180))
                .afterTime(0.9, intake43OpenAction)
                .splineToLinearHeading(new Pose2d(-36, -9.5, Math.toRadians(0)), Math.toRadians(-180))
                .waitSeconds(.2)
                .afterTime(0.6, intake43CloseAction)
                .strafeToLinearHeading(new Vector2d(-44, -10.8), Math.toRadians(0))
                .build();


        Action intake43Traj_Right = robot.drive.actionBuilder(new Pose2d(52, -44, Math.toRadians(0)))
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(24, -9, Math.toRadians(0)), Math.toRadians(-180))


                .afterTime(0.9, intake43OpenActionRight)
                .splineToLinearHeading(new Pose2d(-36.5, -9, Math.toRadians(0)), Math.toRadians(180))
                .waitSeconds(.2)

                .afterTime(0.6, intake43CloseAction)
                .strafeToLinearHeading(new Vector2d(-43.95, -9), Math.toRadians(0))

                .build();


        Action deposit43Traj_Middle = robot.drive.actionBuilder(new Pose2d(-44, -10.8, Math.toRadians(0)))
                .afterTime(0, pleaseFixIntakeFar())
                .afterTime(1, updateElevatorHeight(1600))
                .strafeToLinearHeading(new Vector2d(30, -12), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(52, -28, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .afterTime(0, deposit43Action)
                .build();

        Action deposit43Traj_Left = robot.drive.actionBuilder(new Pose2d(-44.5, -10.5, Math.toRadians(0)))
                .afterTime(0, pleaseFixIntakeFar())
                .afterTime(1, updateElevatorHeight(1600))
                .strafeToLinearHeading(new Vector2d(30, -12), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(52, -30, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .afterTime(0, deposit43Action)
                .build();


        Action deposit43Traj_Right = robot.drive.actionBuilder(new Pose2d(-43.95, -9, Math.toRadians(0)))

                .afterTime(0, pleaseFixIntakeFar())

                .afterTime(1, updateElevatorHeight(1600))

                .strafeToLinearHeading(new Vector2d(30, -12), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(52, -28, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .afterTime(0, deposit43Action)
                .build();


        Action parkTraj = robot.drive.actionBuilder(new Pose2d(-53, -26, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(46, -32), Math.toRadians(90))
                .build();

        ParallelAction placePurplePixelLeft = new ParallelAction(
                placePurpleTrajLeft
        );

        ParallelAction placePurplePixelMiddle = new ParallelAction(
                placePurpleTrajMiddle
        );

        ParallelAction placePurplePixelRight = new ParallelAction(
                placePurpleTrajRight
        );


        SequentialAction intake5Left = new SequentialAction(
                intake5TrajLeft
        );

        SequentialAction intake5Middle = new SequentialAction(
                intake5TrajMiddle
        );

        SequentialAction intake5Right = new SequentialAction(
                intake5TrajRight
        );

        ParallelAction depositPreloadLeft = new ParallelAction(
                depositPreloadTrajLeft

        );

        ParallelAction depositPreloadMiddle = new ParallelAction(
                depositPreloadTrajMiddle

        );

        ParallelAction depositPreloadRight = new ParallelAction(
                depositPreloadTrajRight

        );

        ParallelAction intake43_Left = new ParallelAction(
                intake43Traj_Left
        );

        ParallelAction intake43_Middle = new ParallelAction(
                intake43Traj_Middle
        );

        ParallelAction intake43_Right = new ParallelAction(
                intake43Traj_Right
        );
        ParallelAction deposit43_Left = new ParallelAction(
                deposit43Traj_Left
        );

        ParallelAction deposit43_Middle = new ParallelAction(
                deposit43Traj_Middle
        );

        ParallelAction deposit43_Right = new ParallelAction(
                deposit43Traj_Right

        );
        SequentialAction park = new SequentialAction(
                parkTraj
        );

        blueRightLeft = new SequentialAction(
                placePurplePixelLeft,
                intake5Left,
                depositPreloadLeft,
                park
        );

        blueRightMiddle = new SequentialAction(
                placePurplePixelMiddle,
                intake5Middle,
                depositPreloadMiddle,
                park
        );

        blueRightRight = new SequentialAction(
                placePurplePixelRight,
                intake5Right,
                depositPreloadRight,
                park
        );


        while (opModeInInit() && !isStopRequested()) {
            betterGamepad2.update();
            intake.setAngle(Intake.Angle.MID);
            intakeExtension.setTarget(0);
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addData("POS", propLocation.name());
            telemetry.addData("elevator pos", tempHeight);
            telemetry.update();
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

            if (betterGamepad2.dpadRightOnce()) {
                tempHeight = tempHeightMax;
            } else if (betterGamepad2.dpadLeftOnce()) {
                tempHeight = tempHeightMin;
            }

            if (betterGamepad2.dpadUpOnce()) {
                if (first) {
                    webcam.stopStreaming();
                    first = false;
                }

                propLocation = cycleVision(propLocation);
            } else if (betterGamepad2.dpadDownOnce()) {
                initCamera();
                webcam.setPipeline(propPipelineRedLeft);
            }
        }

        waitForStart();
        webcam.stopStreaming();

        if (isStopRequested()) return;

        time.reset();

        switch (propLocation) {
            case LEFT:
                runBlocking(new ParallelAction(
                        blueRightLeft,
                        updateActions.updateSystems()
                ));
                break;
            case RIGHT:
                runBlocking(new ParallelAction(
                        blueRightRight,
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


    Action updateElevatorHeight(int height) {
        return new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.TELEOP_MID),
                new SleepAction(.75),
                depositActions.readyForDeposit(height));
    }


    SequentialAction pleaseFixIntakeFar() {
        return new SequentialAction(
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                new SleepAction(0.1),
                intakeActions.openExtension(300),
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                new SleepAction(0.5),
                intakeActions.closeExtension(),
                new SleepAction(0.2),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)

        );

    }

    SequentialAction pleaseFixIntakeClose() {
        return new SequentialAction(
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                new SleepAction(0.1),
                intakeActions.openExtension(300),
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                new SleepAction(0.5),
                intakeActions.closeExtension(),
                new SleepAction(0.2),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)

        );

    }


    /*
    SequentialAction fixIntake() {
        return new SequentialAction(


                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(0.1),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new InstantAction(() -> intakeExtension.setAggresive(false)),
                intakeActions.openExtension(75),
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                intakeActions.openExtension(-20),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)

        );


     */
}
