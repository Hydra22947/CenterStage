package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

//import static org.firstinspires.ftc.teamcode.auto.Actions.CheckAprilTagAction.initAprilTag;

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
//import org.firstinspires.ftc.teamcode.auto.Actions.CheckAprilTagAction;
import org.firstinspires.ftc.teamcode.auto.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineBlueLeft;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.jetbrains.annotations.NotNull;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "2+4 - Auto Blue Left")
public class AutoBlueLeftTwoPlusFour extends LinearOpMode {
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
    AprilTagProcessor aprilTagProcessor;

    public enum PropLocation
    {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static PropLocation propLocation = PropLocation.MIDDLE;
    PropPipelineBlueLeft propPipelineBlueLeft;
    OpenCvWebcam webcam;
    public static int tempHeight = 1450;
    public static int minHeight = 1000;


    public static int RIGHT_EXTENSION = 270;
    public static int MIDDLE_EXTENSION = 300;
    public static int LEFT_EXTENSION = 200;

    SequentialAction blueLeftLeft;
    SequentialAction blueLeftMiddle;
    SequentialAction blueLeftRight;
    boolean shouldUseAprilTag = true;

    @Override
    public void runOpMode() {
        BetterGamepad betterGamepad2 = new BetterGamepad(gamepad2);
        propPipelineBlueLeft = new PropPipelineBlueLeft();
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueLeft);

        autoConstants = new AutoConstants();

        initCamera();
        webcam.setPipeline(propPipelineBlueLeft);

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
                intakeActions.moveIntake(Intake.Angle.TELEOP_MID),
                new SleepAction(.2),
                depositActions.moveElevator(minHeight),
                new SleepAction(0.2),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 0),
                depositActions.moveElevator(minHeight+300),
                new SleepAction(0.2),
                new InstantAction(()-> depositActions.retractElevator())




        );
        SequentialAction readyForDepositAction = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),
                new SleepAction(0.5),
                depositActions.readyForDeposit(tempHeight)

        );

        SequentialAction depositSecondCycle = new SequentialAction(

               readyForDepositAction,

                intakeActions.moveIntake(Intake.Angle.TELEOP_MID),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),

                intakeActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(1.45),
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),

                new SleepAction(0.4),
                depositActions.moveElevator(tempHeight + 400),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),
                new SleepAction(0.2),
                depositActions.moveElevator(tempHeight -200),
                new SleepAction(0.3),
                depositActions.retractDeposit()
        );


        SequentialAction placePurplePixelAction_MIDDLE = new SequentialAction(
                new ParallelAction(
                        intakeActions.moveIntake(Intake.Angle.INTAKE),
                        intakeActions.openExtension( MIDDLE_EXTENSION)),
                new SleepAction(0.5),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN,ClawSide.BOTH)
        );

        SequentialAction placePurplePixelAction_LEFT = new SequentialAction(
                new ParallelAction(
                        intakeActions.moveIntake(Intake.Angle.INTAKE),
                        intakeActions.openExtension( LEFT_EXTENSION)),
                new SleepAction(0.5),
               intakeActions.moveIntakeClaw(Intake.ClawState.OPEN,ClawSide.BOTH)
        );
       SequentialAction placePurplePixelAction_RIGHT = new SequentialAction(
               intakeActions.moveIntake(Intake.Angle.INTAKE),
               new SleepAction(0.6),
               new ParallelAction(
               intakeActions.openExtension( RIGHT_EXTENSION)),
               new SleepAction(0.5),
               intakeActions.moveIntakeClaw(Intake.ClawState.OPEN,ClawSide.BOTH)
       );

        SequentialAction retractPurpleAction = new SequentialAction(
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.2),
                new ParallelAction(
                        new InstantAction(() -> intakeExtension.setAggresive(true)),
                        intakeActions.closeExtension(),
                        new SleepAction(0.2),
                        intakeActions.moveIntake(Intake.Angle.MID)
                )
        );

        SequentialAction openIntakeWhitePixelAction54 = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.TOP_54),
                new ParallelAction(
                new SleepAction(.5),
                intakeActions.lock(PlacePurpleActions.CloseClaw.SUPER_RIGHT_CLOSE),
                intakeActions.release(PlacePurpleActions.OpenClaw.LEFT_OPEN)),
                new SleepAction(0.7),
                new InstantAction(()-> intakeExtension.setAggresive(false)),
                intakeActions.openExtension(430 )

        );

        SequentialAction openIntakeWhitePixelAction32 = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.TOP_32),
                new SleepAction(.5),
                intakeActions.lock(PlacePurpleActions.CloseClaw.SUPER_RIGHT_CLOSE),
                intakeActions.release(PlacePurpleActions.OpenClaw.LEFT_OPEN),
                new SleepAction(1),
                new InstantAction(() -> intakeExtension.setAggresive(false)),
                intakeActions.openExtension(430 )

        );


        SequentialAction closeIntakeWhitePixelAction = new SequentialAction(
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(.5),
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                intakeActions.closeExtension(),
                new SleepAction(.1),
                intakeActions.moveIntake(Intake.Angle.AUTO_FIX_INTAKE)

                //returnTransfer()
        );

       SequentialAction placePurplePixelSequence_Right = new SequentialAction(
                depositActions.readyForDeposit(minHeight),
                new SleepAction(0.1),
                placePurplePixelAction_RIGHT,
                retractPurpleAction

        );

        SequentialAction placePurplePixelSequence_Middle = new SequentialAction(
                depositActions.readyForDeposit(minHeight),
                new SleepAction(0.1),
                placePurplePixelAction_MIDDLE,
                retractPurpleAction

        );

        SequentialAction placePurplePixelSequence_Left = new SequentialAction(
                depositActions.readyForDeposit(minHeight),
                new SleepAction(0.1),
                placePurplePixelAction_LEFT,
                retractPurpleAction

        );

        SequentialAction  intake54Action = new SequentialAction(
                openIntakeWhitePixelAction54,
                new SleepAction(2),
                closeIntakeWhitePixelAction
        );


        
        SequentialAction intake32Action = new SequentialAction(
                openIntakeWhitePixelAction32,
                new SleepAction(1.5),
                returnLockTop32()
        );

        Action placePurpleTraj_RIGHT = robot.drive.actionBuilder(robot.drive.pose)
                .afterTime(0.5 ,placePurplePixelSequence_Right)
                .strafeToLinearHeading(new Vector2d(20 ,34.25), Math.toRadians(0))
                .build();

        Action placeYellowTraj_RIGHT = robot.drive.actionBuilder(new Pose2d(20, 34.25, Math.toRadians(0)))
                .afterTime(1 , depositBlue)
                .splineToConstantHeading(new Vector2d(50.5,28) , Math.toRadians(0))
                .build();


        Action goForIntakeTop54 = robot.drive.actionBuilder(new Pose2d(50.5, 28 , Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(48,30.25) , Math.toRadians(0))
                .setTangent(190)
                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .afterTime(2.3, intake54Action)
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-38.5, 43.6, Math.toRadians(12)), Math.toRadians(-180))
                .strafeTo(new Vector2d(-47.5, 44))
                .waitSeconds(0.2)
                .build();


        Action goForIntakeTop32 = robot.drive.actionBuilder(new Pose2d(50.5, 39 , Math.toRadians(0)))
                .strafeTo(new Vector2d(48,43 ))
                .setTangent(190)
                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .afterTime(2.3, intake32Action)
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-39.5, 43.6, Math.toRadians(15)), Math.toRadians(-180))
                .strafeTo(new Vector2d(-48.5,  44.4))
                .waitSeconds(0.2)
                .build();

        VelConstraint baseVelConstraint = new VelConstraint() {
            @Override
            public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                if (pose2dDual.position.x.value() > 35 && pose2dDual.position.x.value() < 39) {
                    return 5;
                } else {
                    return 50.0;
                }
            }
        };

        Action goPlaceWhiteRIGHT_OR_TOP54 =// new CheckAprilTagAction(shouldUseAprilTag, time,
                robot.drive.actionBuilder(new Pose2d(-47.5,44.5,Math.toRadians(15)))
                        .setTangent(Math.toRadians(90))
                        .afterTime(0,returnFixintake())
                        .splineToConstantHeading(new Vector2d(-32, 58), Math.toRadians(0))
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(22, 58, Math.toRadians(-30)), Math.toRadians(0))
//                        .afterTime(1, new ParallelAction(depositSecondCycle,
//                                new InstantAction(() -> CheckAprilTagAction.turnOnDetection())))
                        .afterTime(1, depositSecondCycle)
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(50.5, 40, Math.toRadians(0)), Math.toRadians(20)/*, baseVelConstraint*/).build();//,
//                new SequentialAction(robot.drive.actionBuilder(new Pose2d(50.5, 40, Math.toRadians(0)))
//                        .strafeToLinearHeading(new Vector2d(50.5, 60), Math.toRadians(-90)).build(),
//                        new InstantAction(() -> intake.move(Intake.Angle.TELEOP_MID)),
//                        new InstantAction(() -> claw.setBothClaw(Claw.ClawState.CLOSED)),
//                        new InstantAction(() -> outtake.setAngle(Outtake.Angle.INTAKE)),
//                        new InstantAction(() -> elevator.setTarget(0)),
//                        new SleepAction(1),
//                        new InstantAction(() -> requestOpModeStop()))
                //);

        Action goPlaceWhiteRIGHT_OR_TOP32 = //new CheckAprilTagAction(shouldUseAprilTag, time,
                robot.drive.actionBuilder(new Pose2d(-48.5, 44.4,Math.toRadians(15)))
                .setTangent(Math.toRadians(90))
                .afterTime(0,returnFixintake())
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(22, 58), Math.toRadians(0))
                .afterTime(0, returnDepositTop32())
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50.5, 40), Math.toRadians(0))
                        .build();

//                new SequentialAction(robot.drive.actionBuilder(new Pose2d(50.5, 40, Math.toRadians(0)))
//                        .strafeToLinearHeading(new Vector2d(50.5, 60), Math.toRadians(-90)).build(),
//                        new InstantAction(() -> intake.move(Intake.Angle.TELEOP_MID)),
//                        new InstantAction(() -> claw.setBothClaw(Claw.ClawState.CLOSED)),
//                        new InstantAction(() -> outtake.setAngle(Outtake.Angle.INTAKE)),
//                        new InstantAction(() -> elevator.setTarget(0)),
//                        new SleepAction(1),
//                        new InstantAction(() -> requestOpModeStop())
//                );

        Action placePurpleTraj_LEFT = robot.drive.actionBuilder(robot.drive.pose)
                .afterTime(1 ,placePurplePixelSequence_Left)
                .strafeToLinearHeading(new Vector2d(40 ,34.25), Math.toRadians(0))
                .build();

        Action placeYellowTraj_LEFT = robot.drive.actionBuilder(new Pose2d(40, 34.25, Math.toRadians(0)))
                .afterTime(0.8 , depositBlue)
                .strafeTo(new Vector2d(50,39.5))
                .waitSeconds(0.45)
                .strafeTo(new Vector2d(48,43 ))
                .build();


        Action goForIntakeTop54_LEFT_OR_MIDDLE = robot.drive.actionBuilder(new Pose2d(48, 43 , Math.toRadians(0)))
                .setTangent(190)
                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .afterTime(0.2, intake54Action)
                .splineToLinearHeading(new Pose2d(-38.5, 37.3, Math.toRadians(0)), Math.toRadians(-180))
                .strafeTo(new Vector2d(-44.5, 37.4))
                .waitSeconds(0.2)
                //.afterTime(2.3, intake54Action)
                .build();


        Action goForIntakeTop32_LEFT_OR_MIDDLE = robot.drive.actionBuilder(new Pose2d(48, 43, Math.toRadians(0)))
                .setTangent(190)
                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .afterTime(0.2, intake32Action)
                .splineToLinearHeading(new Pose2d(-39.5, 37.3, Math.toRadians(0)), Math.toRadians(-180))
                .strafeTo(new Vector2d(-46, 38))
                .waitSeconds(0.2)
                .build();

        Action goPlaceWhite_LEFT_OR_MIDDLE54 = robot.drive.actionBuilder(new Pose2d(-44.5, 37.4,Math.toRadians(15)))
                .setTangent(Math.toRadians(90))
                .afterTime(0,returnFixintake())
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(22, 58), Math.toRadians(0))
                .afterTime(0, depositSecondCycle)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50.5, 39), Math.toRadians(0))
                .waitSeconds(0.1)
                .strafeTo(new Vector2d(48,43 ))
                .build();

        Action goPlaceWhite_LEFT_OR_MIDDLE32 = robot.drive.actionBuilder(new Pose2d(-47.5, 38,Math.toRadians(15)))

                .setTangent(Math.toRadians(90))
                .afterTime(0,returnFixintake())
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(22, 58), Math.toRadians(0))
                .afterTime(0, returnDepositTop32())
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50.5, 39), Math.toRadians(0))
                .build();


        Action placePurpleTraj_MIDDLE = robot.drive.actionBuilder(robot.drive.pose)
                .afterTime(1 , placePurplePixelSequence_Middle)
                .strafeToLinearHeading(new Vector2d(30 ,26), Math.toRadians(0))
                .build();

        Action placeYellowTraj_MIDDLE = robot.drive.actionBuilder(new Pose2d(30, 26, Math.toRadians(0)))
                .afterTime(0.8 , depositBlue)
                .strafeTo(new Vector2d(50,34.25))
                .waitSeconds(0.45)
                .strafeTo(new Vector2d(48,43 ))
                .build();

/*
        Action goForIntakeTop54_MIDDLE = robot.drive.actionBuilder(new Pose2d(50, 34.25, Math.toRadians(0)))
                .setTangent(190)
                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .afterTime(2.3, intake54Action)
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-38.5, 37.3, Math.toRadians(0)), Math.toRadians(-180))
                .strafeTo(new Vector2d(-46, 37.3))
                .waitSeconds(0.2)
                .build();


 */
        Action goPark_Right = robot.drive.actionBuilder(new Pose2d( 50.5 , 40  , Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(46 , 34) ,  Math.toRadians(-90))
                .build();

        Action goPark_LeftOrMiddle= robot.drive.actionBuilder(new Pose2d( 50.5 , 39  , Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(46 , 34) ,  Math.toRadians(-90))
                .build();

        // Right Traj
        ParallelAction placePurplePixel_Right = new ParallelAction(
                placePurpleTraj_RIGHT
        );
        ParallelAction placePreloadOnBoard_Right = new ParallelAction(
                placeYellowTraj_RIGHT

        );
        ParallelAction intake54_Right = new ParallelAction(
                goForIntakeTop54

        );
        ParallelAction intake32_Right = new ParallelAction(
                goForIntakeTop32

        );
        ParallelAction deposit54_Right = new ParallelAction(
                goPlaceWhiteRIGHT_OR_TOP54

        );
        ParallelAction deposit32_Right = new ParallelAction(
                goPlaceWhiteRIGHT_OR_TOP32

        );


        // Left Traj
        ParallelAction placePurplePixel_Left = new ParallelAction(
                placePurpleTraj_LEFT
        );
        ParallelAction placePreloadOnBoard_Left = new ParallelAction(
                placeYellowTraj_LEFT

        );
        ParallelAction intake54_LeftOrMiddle = new ParallelAction(
                goForIntakeTop54_LEFT_OR_MIDDLE

        );
        ParallelAction intake32_LeftOrMiddle = new ParallelAction(
                goForIntakeTop32_LEFT_OR_MIDDLE

        );
        ParallelAction deposit54_LeftOrMiddle = new ParallelAction(
                goPlaceWhite_LEFT_OR_MIDDLE54

        );
        ParallelAction deposit32_LeftOrMiddle = new ParallelAction(
                goPlaceWhite_LEFT_OR_MIDDLE32

        );



        // Middle Traj

        ParallelAction placePurplePixel_Middle = new ParallelAction(
                placePurpleTraj_MIDDLE
        );
        ParallelAction placePreloadOnBoard_Middle = new ParallelAction(
                placeYellowTraj_MIDDLE

        );



        blueLeftRight= new SequentialAction(
                placePurplePixel_Right
                , placePreloadOnBoard_Right
                , intake54_Right
                , deposit54_Right
                , intake32_Right
                , deposit32_Right
                , goPark_Right

        );

        blueLeftLeft= new SequentialAction(
                placePurplePixel_Left
                , placePreloadOnBoard_Left
                , intake54_LeftOrMiddle
                , deposit54_LeftOrMiddle
                , intake32_LeftOrMiddle
                , deposit32_LeftOrMiddle
                , goPark_LeftOrMiddle

        );

        blueLeftMiddle = new SequentialAction(
                placePurplePixel_Middle
                , placePreloadOnBoard_Middle
                , intake54_LeftOrMiddle
                , deposit54_LeftOrMiddle
                , intake32_LeftOrMiddle
                , deposit32_LeftOrMiddle
                , goPark_LeftOrMiddle

        );

        while (opModeInInit() && !isStopRequested()) {
            betterGamepad2.update();

            intake.setAngle(Intake.Angle.MID);
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addData("POS", propPipelineBlueLeft.getLocation());
            telemetry.addData("use april tag? (a - yes, y - no)", shouldUseAprilTag);

            switch (propPipelineBlueLeft.getLocation()) {
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

            if(betterGamepad2.AOnce())
            {
                shouldUseAprilTag = true;
            }
            else if(betterGamepad2.YOnce())
            {
                shouldUseAprilTag = false;
            }

            telemetry.addLine("Initialized");
            telemetry.update();
        }

        waitForStart();

        webcam.stopStreaming();
        if (isStopRequested()) return;

        time.reset();

        //initAprilTag();

        switch (propLocation)
        {
            case LEFT:
                runBlocking(new ParallelAction(
                        blueLeftLeft,
                        updateActions.updateSystems()
                ));
                break;
            case MIDDLE:
                runBlocking(new ParallelAction(
                        blueLeftMiddle,
                        updateActions.updateSystems()
                ));
                break;
            case RIGHT:
                runBlocking(new ParallelAction(
                        blueLeftRight,
                        updateActions.updateSystems()));
                break;
        }
    }


    SequentialAction returnLockTop32 ()
    {
        return  new SequentialAction(
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(.45 ),
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                intakeActions.closeExtension(),
                new SleepAction(.1),
                intakeActions.moveIntake(Intake.Angle.AUTO_FIX_INTAKE)

              //  returnTransfer()

        );
    }
    SequentialAction returnDepositTop32 ()
    {
        return  new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                depositActions.readyForDeposit(tempHeight + 400),

                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),

                intakeActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(1),
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),
                new SleepAction(0.2),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),
                new SleepAction(0.3),
                depositActions.moveElevator(tempHeight + 950),
                depositActions.retractDeposit()
        );
    }


    SequentialAction returnFixintake () {
        return new SequentialAction(
                new SleepAction(.5),
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
                intakeActions.closeExtension(),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
                new SleepAction(0.2),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(0.2),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)

        );
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