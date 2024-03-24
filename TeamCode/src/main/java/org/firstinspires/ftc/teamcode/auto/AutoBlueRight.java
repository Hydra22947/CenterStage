package org.firstinspires.ftc.teamcode.auto;

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
import org.firstinspires.ftc.teamcode.auto.MaxAuto.AutoBlueRightMiddleMax;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineBlueRight;
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineRedRight;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.jetbrains.annotations.NotNull;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "2+3 - Auto Right Blue")
public class AutoBlueRight extends LinearOpMode {
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
    PropPipelineBlueRight propPipelineBlueRight;
    OpenCvWebcam webcam;

    SequentialAction blueRightLeft, blueRightMiddle, blueRightRight;
    SequentialAction readyForDepositAction;
    int tempHeight;

    @Override
    public void runOpMode() {
        BetterGamepad betterGamepad2 = new BetterGamepad(gamepad2);
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
        intakeActions = new PlacePurpleActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);

        tempHeight = 1250;

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
                new SleepAction(1.4),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1500),
                new SleepAction(.5),
                depositActions.moveElevator(tempHeight + 300),
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
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 0),

                new SleepAction(0.25),
                depositActions.moveElevator(tempHeight + 300),
                depositActions.retractDeposit());

        //Trajectories

        Action placePurpleTrajLeft = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-29, 37.75), Math.toRadians(-40))
                .build();

        Action placePurpleTrajMiddle = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-32, 33), Math.toRadians(-90))
                .build();

        Action placePurpleTrajRight = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-48, 43), Math.toRadians(-90))
                .build();

        Action intake5TrajLeft = robot.drive.actionBuilder(new Pose2d(-29, 37.75, Math.toRadians(-40)))
                .strafeToLinearHeading(new Vector2d(-32, 40), Math.toRadians(-40))
                .setTangent(-90)
                .splineToLinearHeading(new Pose2d(-45.5, 22, Math.toRadians(0)), Math.toRadians(180))
                .stopAndAdd(intake5OpenAction)
                .strafeToLinearHeading(new Vector2d(-54.2, 22 ),  Math.toRadians(0))
                .stopAndAdd(intake5CloseAction)
                .build();

        Action intake5TrajMiddle = robot.drive.actionBuilder(new Pose2d(-32, 33, Math.toRadians(-90)))
                .setTangent(-180)
                .splineToSplineHeading(new Pose2d(-48, 41, Math.toRadians(0)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-46, 23, Math.toRadians(0)), Math.toRadians(180))
                .afterTime(0, intake5OpenAction)
                .waitSeconds(0.25)
                .strafeToLinearHeading(new Vector2d(-53, 23), Math.toRadians(0))
                .stopAndAdd(intake5CloseAction)
                .strafeToLinearHeading(new Vector2d(-48, 23), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-32 ,10) ,Math.toRadians(0))
                .build();

        Action intake5TrajRight = robot.drive.actionBuilder(new Pose2d(-50, 43, Math.toRadians(-90)))
                .strafeToSplineHeading(new Vector2d(-42, 45), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-38, 11, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-45, 11), Math.toRadians(0))
                .stopAndAdd(intake5OpenAction)
                .strafeToLinearHeading(new Vector2d(-53, 11), Math.toRadians(0))
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

        Action depositPreloadTrajLeft = robot.drive.actionBuilder(new Pose2d(-55.2, 22, Math.toRadians(0)))
                .splineToLinearHeading( new Pose2d(-42, 10, Math.toRadians(0)), Math.toRadians(0))
                //deposit
                .afterTime(0, pleaseFixIntake())
                .splineToLinearHeading(new Pose2d(15, 8,Math.toRadians(0)), Math.toRadians(0))
                .afterTime(0.85, readyForDepositAction)
                .splineToSplineHeading(new Pose2d(31, 20, Math.toRadians(24.5)), Math.toRadians(60), baseVelConstraint)
                .splineToLinearHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .afterTime(0, depositAction)
                .strafeToSplineHeading(new Vector2d(51, 38.2), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(51, 34), Math.toRadians(0))
                .build();

        Action depositPreloadTrajMiddle = robot.drive.actionBuilder(new Pose2d(-32, 10, Math.toRadians(0)))
                //deposit
                .afterTime(0.5, pleaseFixIntake())
                .splineToLinearHeading(new Pose2d(15,8 ,Math.toRadians(0)), Math.toRadians(0))
                .afterTime(0.85, readyForDepositAction)
                .splineToSplineHeading(new Pose2d(31, 20, Math.toRadians(24.5)), Math.toRadians(60), baseVelConstraint)
                .splineToLinearHeading(new Pose2d(52, 26.5, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .afterTime(0, depositAction)
                .strafeToSplineHeading(new Vector2d(52, 34), Math.toRadians(0))
                .build();

        Action depositPreloadTrajRight = robot.drive.actionBuilder(new Pose2d(-52.5, 11, Math.toRadians(0)))
                .afterTime(0.5, pleaseFixIntake())
                .splineToLinearHeading(new Pose2d(15, 8,Math.toRadians(0)), Math.toRadians(0))
                .afterTime(0.85, readyForDepositAction)
                .splineToSplineHeading(new Pose2d(31, 20, Math.toRadians(24.5)), Math.toRadians(60), baseVelConstraint)
                .splineToLinearHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .afterTime(0, depositAction)
                .strafeToSplineHeading(new Vector2d(51, 24.5), Math.toRadians(0))
                .build();

        Action intake43Traj_MiddleOrLeft = robot.drive.actionBuilder(new Pose2d(51, 34, Math.toRadians(0)))
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(24, 9, Math.toRadians(0)), Math.toRadians(180))


                .afterTime(0.9, intake43OpenAction)
                .splineToLinearHeading(new Pose2d(-36, 9.65, Math.toRadians(0)), Math.toRadians(-180))
                .waitSeconds(.2)

                .afterTime(0.6, intake43CloseAction)
                .strafeToLinearHeading(new Vector2d(-43.8, 10.8), Math.toRadians(0))

                .build();


        Action intake43Traj_Right = robot.drive.actionBuilder(new Pose2d(52, 25, Math.toRadians(0)))
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(24, 9, Math.toRadians(0)), Math.toRadians(180))


                .afterTime(0.9, intake43OpenActionRight)
                .splineToLinearHeading(new Pose2d(-36, 9.65, Math.toRadians(0)), Math.toRadians(-180))
                .waitSeconds(.2)

                .afterTime(0.6, intake43CloseAction)
                .strafeToLinearHeading(new Vector2d(-43.95, 11.9), Math.toRadians(0))

                .build();


        Action deposit43Traj_MiddleOrLeft = robot.drive.actionBuilder(new Pose2d(-44.5, 10.5, Math.toRadians(0)))

                .afterTime(0, pleaseFixIntake())


                .strafeToLinearHeading(new Vector2d(15, 8), Math.toRadians(0))
                .afterTime(0.85, updateElevatorHeight(1600))
                .splineToSplineHeading(new Pose2d(31, 20, Math.toRadians(24.5)), Math.toRadians(60), baseVelConstraint)
                .splineToLinearHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0))
                .afterTime(0, deposit43Action)
                .build();

        Action deposit43Traj_Right = robot.drive.actionBuilder(new Pose2d(-44.5, 11.9, Math.toRadians(0)))

                .afterTime(0, pleaseFixIntake())


                .strafeToLinearHeading(new Vector2d(15, 8), Math.toRadians(0))
                .afterTime(0.85, updateElevatorHeight(1600))
                .splineToSplineHeading(new Pose2d(31, 20, Math.toRadians(24.5)), Math.toRadians(60), baseVelConstraint)
                .splineToLinearHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0))
                .afterTime(0, deposit43Action)
                .build();

        Action parkTraj = robot.drive.actionBuilder(new Pose2d(52.5, 28, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(46, 32), Math.toRadians(-90))
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

        ParallelAction intake43_MiddleOrLeft = new ParallelAction(
                intake43Traj_MiddleOrLeft
        );

        ParallelAction intake43_Right= new ParallelAction(
                intake43Traj_Right
        );
        ParallelAction deposit43_MiddleOrLeft = new ParallelAction(
                deposit43Traj_MiddleOrLeft

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
                intake43Traj_MiddleOrLeft,
                deposit43_MiddleOrLeft,
                park
        );

        blueRightMiddle = new SequentialAction(
                placePurplePixelMiddle,
                intake5Middle,
                depositPreloadMiddle,
                intake43_MiddleOrLeft,
                deposit43_MiddleOrLeft,
                park
        );

        blueRightRight = new SequentialAction(
                placePurplePixelRight,
                intake5Right,
                depositPreloadRight,
                intake43_Right,
                deposit43_Right,
                park
        );


        while (opModeInInit() && !isStopRequested()) {

            betterGamepad2.update();

            intake.setAngle(Intake.Angle.MID);

            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addData("POS", propPipelineBlueRight.getLocation());
            telemetry.addData("elevator pos", tempHeight);

            switch (propPipelineBlueRight.getLocation()) {
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

            telemetry.addLine("Initialized");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

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
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                new SleepAction(0.1),
                intakeActions.openExtension(350),
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                new SleepAction(0.4),
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
