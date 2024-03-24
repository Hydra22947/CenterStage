package org.firstinspires.ftc.teamcode.auto.MaxAuto;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoSettings.writeToFile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineRedRight;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.jetbrains.annotations.NotNull;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "2+3 - Auto Right Left Blue MAX")
public class AutoBlueRightLeftMax extends LinearOpMode {
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

    public static AutoBlueRightMiddleMax.PropLocation propLocation = AutoBlueRightMiddleMax.PropLocation.MIDDLE;
    PropPipelineRedRight propPipelineRedRight;
    OpenCvWebcam webcam;

    SequentialAction blueRightMiddle;
    SequentialAction readyForDepositAction;
    int tempHeight;

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

        tempHeight = 1400;

        SequentialAction intake5OpenAction = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH)

        );

        SequentialAction intake5CloseAction = new SequentialAction(
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
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


        SequentialAction intake43CloseAction = new SequentialAction(
                new SleepAction(.2),
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(.5),
                intakeActions.closeExtension(),
                new SleepAction(.5),
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),


                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(.5),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.25),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)


        );

        SequentialAction depositAction = new SequentialAction(

                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 0),
                new SleepAction(1.6),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1500),
                new SleepAction(.5),
                depositActions.moveElevator(tempHeight),
                depositActions.retractDeposit()
        );

        readyForDepositAction = new SequentialAction(
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
                intakeActions.moveIntake(Intake.Angle.TELEOP_MID),
                new SleepAction(.75),
                depositActions.readyForDeposit(tempHeight));


        SequentialAction deposit43Action = new SequentialAction(
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),
                new SleepAction(1),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 0),

                new SleepAction(0.25),
                depositActions.moveElevator(tempHeight),
                depositActions.retractDeposit());

        //Trajectories

        Action placePurpleTraj = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-30, 40), Math.toRadians(-40))
                .build();

        Action intake5Traj = robot.drive.actionBuilder(new Pose2d(-30, 40, Math.toRadians(-40)))
                .strafeToLinearHeading(new Vector2d(-28, 40), Math.toRadians(-40))
                .setTangent(-90)
                .splineToLinearHeading(new Pose2d(-46, 22, Math.toRadians(0)), Math.toRadians(180))
                .stopAndAdd(intake5OpenAction)

                .splineToLinearHeading(new Pose2d(-54.25, 22 ,  Math.toRadians(0)), Math.toRadians(0))
                .stopAndAdd(intake5CloseAction)


                .build();

        VelConstraint baseVelConstraint = new VelConstraint() {
            @Override
            public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                if (pose2dDual.position.x.value() > 30 && pose2dDual.position.x.value() < 37.5) {
                    return 5;
                } else {
                    return 50.0;
                }
            }
        };

        Action depositPreloadTraj = robot.drive.actionBuilder(new Pose2d(-54.25, 22, Math.toRadians(0)))

                .splineToLinearHeading( new Pose2d(-42, 10, Math.toRadians(0)), Math.toRadians(0))

                //deposit
                .afterTime(0, pleaseFixIntake())
                .splineToLinearHeading(new Pose2d(15, 8,Math.toRadians(0)), Math.toRadians(0))
                .afterTime(1, readyForDepositAction)

                .splineToSplineHeading(new Pose2d(31, 20, Math.toRadians(24.5)), Math.toRadians(60), baseVelConstraint)

                .splineToLinearHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .afterTime(0, depositAction)

                .strafeToSplineHeading(new Vector2d(51, 38), Math.toRadians(0))


                .build();

        Action intake43Traj = robot.drive.actionBuilder(new Pose2d(52, 40, Math.toRadians(0)))
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(24, 9.5, Math.toRadians(0)), Math.toRadians(180))


                .afterTime(0.9, intake43OpenAction)
                .splineToLinearHeading(new Pose2d(-36, 9.5, Math.toRadians(0)), Math.toRadians(-180))
                .waitSeconds(.2)

                .afterTime(0.6, intake43CloseAction)
                .strafeToLinearHeading(new Vector2d(-45, 9.5), Math.toRadians(0))

                .build();



        Action deposit43Traj = robot.drive.actionBuilder(new Pose2d(-44.75, 9, Math.toRadians(0)))

                .afterTime(0, pleaseFixIntake())


                .strafeToLinearHeading(new Vector2d(15, 8), Math.toRadians(0))
                .afterTime(1, updateElevatorHeight(1700))
                .splineToSplineHeading(new Pose2d(31, 20, Math.toRadians(24.5)), Math.toRadians(60), baseVelConstraint)
                .splineToLinearHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0))
                .afterTime(0, deposit43Action)
                .build();

        Action parkTraj = robot.drive.actionBuilder(new Pose2d(52.5, 28, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(46, 32), Math.toRadians(-90))
                .build();

        ParallelAction placePurplePixel = new ParallelAction(
                placePurpleTraj
        );


        SequentialAction intake5 = new SequentialAction(
                intake5Traj
        );

        ParallelAction depositPreload = new ParallelAction(
                depositPreloadTraj

        );

        ParallelAction intake43 = new ParallelAction(
                intake43Traj
        );
        ParallelAction deposit43 = new ParallelAction(
                deposit43Traj

        );
        SequentialAction park = new SequentialAction(
                parkTraj
        );

        blueRightMiddle = new SequentialAction(
                placePurplePixel,
                intake5,
                depositPreload,
                intake43,
                deposit43,
                park
        );


        while (opModeInInit() && !isStopRequested()) {

            betterGamepad2.update();

            intake.setAngle(Intake.Angle.MID);

            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addData("POS", propPipelineRedRight.getLocation());
//            telemetry.addData("NO PROP", propPipelineRedRight.NO_PROP);
            switch (propPipelineRedRight.getLocation()) {
                case Left:
                    propLocation = AutoBlueRightMiddleMax.PropLocation.LEFT;
                    break;
                case Right:
                    propLocation = AutoBlueRightMiddleMax.PropLocation.RIGHT;
                    break;
                case Center:
                    propLocation = AutoBlueRightMiddleMax.PropLocation.MIDDLE;
                    break;
            }

            if (betterGamepad2.dpadUpOnce()) {
//                propPipelineRedRight.NO_PROP++;
            } else if (betterGamepad2.dpadDownOnce()) {
//                propPipelineRedRight.NO_PROP--;
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


    Action updateElevatorHeight(int height) {
        return new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.TELEOP_MID),
                new SleepAction(.75),
                depositActions.readyForDeposit(height));
    }


    SequentialAction pleaseFixIntake() {
        return new SequentialAction(
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new InstantAction(() -> intakeExtension.setAggresive(false)),
                new SleepAction(0.1),
                intakeActions.openExtension(100),
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                new SleepAction(0.1),
                intakeActions.closeExtension(),
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
