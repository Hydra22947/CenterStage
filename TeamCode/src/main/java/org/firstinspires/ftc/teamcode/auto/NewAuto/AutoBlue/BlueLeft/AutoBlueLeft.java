package org.firstinspires.ftc.teamcode.auto.NewAuto.AutoBlue.BlueLeft;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineBlueLeft;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "Test New Auto Blue Left" , group = "AutoBlue")
public class AutoBlueLeft extends LinearOpMode {
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

    public static int PIXEL_EXTENSION = 500;

    public static AutoSettings.PropLocation propLocation = AutoSettings.PropLocation.MIDDLE;
    PropPipelineBlueLeft propPipelineBlueLeft;
    OpenCvWebcam webcam;
    boolean vision = true;

    public static int tempHeight = 900;
    boolean first = true;

    SequentialAction runMiddle;
    SequentialAction runRight;
    SequentialAction runLeft;

    @Override
    public void runOpMode() {
        first = true;
        BetterGamepad betterGamepad2 = new BetterGamepad(gamepad2);
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        propPipelineBlueLeft = new PropPipelineBlueLeft();
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

        depositActions = new DepositActions(elevator, intake, claw, outtake , intakeExtension);
        intakeActions = new IntakeActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);



        SequentialAction placePurplePixelAction_Middle = new SequentialAction(
                new ParallelAction(
                        intakeActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(0.5)),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN,ClawSide.BOTH)
        );

        SequentialAction placePurplePixelAction_Left = new SequentialAction(
                new ParallelAction(
                        intakeActions.moveIntake(Intake.Angle.INTAKE),
                        new SleepAction(0.6)),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN,ClawSide.BOTH)
        );


        SequentialAction placePurplePixelAction_Right = new SequentialAction(
                new ParallelAction(
                        intakeActions.moveIntake(Intake.Angle.INTAKE),
                        new SleepAction(0.3)),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN,ClawSide.BOTH)
        );

        SequentialAction retractPurpleAction = new SequentialAction(
                new ParallelAction(
                        new InstantAction(() -> intakeExtension.setAggresive(true)),
                        new InstantAction(() -> intakeActions.openExtension(0)),
                        new SleepAction(0.2),
                        intakeActions.moveIntake(Intake.Angle.AUTO_MID)
                )
        );

        SequentialAction placePurplePixelSequence_Middle = new SequentialAction(
                placePurplePixelAction_Middle,
                retractPurpleAction

        );

        SequentialAction placePurplePixelSequence_Right = new SequentialAction(
                placePurplePixelAction_Right,
                retractPurpleAction

        );

        SequentialAction placePurplePixelSequence_Left = new SequentialAction(
                placePurplePixelAction_Left,
                retractPurpleAction

        );

        SequentialAction placeYellowPixel = new SequentialAction(
                depositActions.moveOuttake(Outtake.Angle.OUTTAKE),
                new SleepAction(0.5),
                depositActions.placePixel(),
                new SleepAction(1),
                depositActions.moveOuttake(Outtake.Angle.INTAKE)

        );



        Action trajBlueLeft =
                robot.drive.actionBuilder(robot.drive.pose)
                        .afterTime(1.2, placePurplePixelSequence_Left)
                        .splineToLinearHeading(new Pose2d(30, 37, Math.toRadians(0)), Math.toRadians(0))
                        .afterTime(0, placeYellowPixel)
                        .splineToSplineHeading(new Pose2d(48, 40,Math.toRadians(0)), Math.toRadians(0))


                        .setTangent(110)
                        .splineToLinearHeading(new Pose2d(20, 58 , Math.toRadians(0)), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(0)), Math.toRadians(180))
                        .afterTime(2.3, IntakePixels54())
                        .splineToLinearHeading(new Pose2d(-50, 37, Math.toRadians(0)), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-54.8, 37, Math.toRadians(0)), Math.toRadians(180))

                        //deposit
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(0)), Math.toRadians(0))

                        .splineToSplineHeading(new Pose2d(20, 58, Math.toRadians(0)), Math.toRadians(0))
                        .afterTime(2.5 , prepOuttake())
                        .splineToLinearHeading(new Pose2d(50, 40, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .afterTime(0 , depositActions.moveClaw(Claw.ClawState.OPEN,ClawSide.BOTH))


                        .build();



        Action trajBlueMiddle =
                robot.drive.actionBuilder(robot.drive.pose)
                        .afterTime(1.3, placePurplePixelSequence_Middle)
                        .splineToLinearHeading(new Pose2d(30, 25,Math.toRadians(0)), Math.toRadians(0))
                        .afterTime(0.2, placeYellowPixel)
                        .splineToSplineHeading(new Pose2d(48, 35,Math.toRadians(0)), Math.toRadians(0))


                        .setTangent(110)
                        .splineToLinearHeading(new Pose2d(20, 58 , Math.toRadians(0)), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(0)), Math.toRadians(180))
                        .afterTime(1.5, IntakePixels54())
                        .splineToLinearHeading(new Pose2d(-50, 37, Math.toRadians(0)), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-53, 37.2, Math.toRadians(0)), Math.toRadians(180))

                        //deposit
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(0)), Math.toRadians(0))

                        .splineToSplineHeading(new Pose2d(20, 58, Math.toRadians(0)), Math.toRadians(0))
                        .afterTime(2.3, prepOuttake())
                        .splineToLinearHeading(new Pose2d(50, 40, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .afterTime(0 , depositActions.moveClaw(Claw.ClawState.OPEN,ClawSide.BOTH))

                        .build();




        Action trajBlueRight =
                robot.drive.actionBuilder(robot.drive.pose)

                        .afterTime(1.01, placePurplePixelSequence_Right)
                        .splineToLinearHeading(new Pose2d(15, 35, Math.toRadians(0)), Math.toRadians(-90))
                        .afterTime(0.85, placeYellowPixel)
                        .splineToSplineHeading(new Pose2d(48, 32,Math.toRadians(0)), Math.toRadians(0))

                        .setTangent(110)
                        .splineToLinearHeading(new Pose2d(20, 58 , Math.toRadians(0)), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(0)), Math.toRadians(180))
                        .afterTime(1.5, IntakePixels54())
                        .splineToLinearHeading(new Pose2d(-50, 37, Math.toRadians(0)), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-53, 37, Math.toRadians(0)), Math.toRadians(180))

                        //deposit
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(0)), Math.toRadians(0))


                        .splineToSplineHeading(new Pose2d(20, 58, Math.toRadians(0)), Math.toRadians(0))
                        .afterTime(2.3 , prepOuttake())
                        .splineToLinearHeading(new Pose2d(50, 40, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .afterTime(0 , depositActions.moveClaw(Claw.ClawState.OPEN,ClawSide.BOTH))


                        .build();





         runMiddle = new SequentialAction(trajBlueMiddle);
         runRight = new SequentialAction(trajBlueRight);
         runLeft = new SequentialAction(trajBlueLeft);

        while (opModeInInit() && !isStopRequested()) {

            betterGamepad2.update();

            intake.setAngle(Intake.Angle.AUTO_MID);

            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addData("POS", propLocation.name());


            if(vision)
            {
                switch (propPipelineBlueLeft.getLocation())
                {
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

            if(betterGamepad2.dpadUpOnce())
            {
                if(first)
                {
                    webcam.stopStreaming();
                    first = false;
                }
                vision = false;

                propLocation = cycleVision(propLocation);
            }

            telemetry.addLine("Initialized");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        webcam.stopStreaming();



        switch (propLocation)
        {
            case LEFT:
                runBlocking(new ParallelAction(
                        runLeft,
                        updateActions.updateSystems()));
                break;
            case RIGHT:
                runBlocking(new ParallelAction(
                        runLeft,
                        updateActions.updateSystems()));
                break;
            case MIDDLE:
                runBlocking(new ParallelAction(
                      runLeft,
                      updateActions.updateSystems()));
                break;


        }

    }

    void initCamera()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });



    }

    SequentialAction IntakePixels54()
    {
        return new SequentialAction(
                new ParallelAction(
                        new InstantAction(()-> intakeActions.moveIntakeClaw(Intake.ClawState.OPEN,ClawSide.BOTH)),
                        intakeActions.moveIntake(Intake.Angle.TOP_54),
                        new SleepAction(0.3),
                        new InstantAction(()-> intakeExtension.setAggresive(true)),
                        new InstantAction(()-> intakeExtension.setTarget(100)),
                        new InstantAction(()-> intakeExtension.setPidControl())

                ),
                new SleepAction(0.3),
                intakeActions.moveIntakeClaw(Intake.ClawState.CLOSE, ClawSide.BOTH),
                intakeActions.moveIntakeClaw(Intake.ClawState.CLOSE, ClawSide.BOTH),
                new SleepAction(1),
                new InstantAction(() -> intakeExtension.setAggresive(false)),
                intakeActions.closeExtension(),
                depositActions.moveOuttake(Outtake.Angle.ALMOST_INTAKE),
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.6),
                new InstantAction(()-> outtake.setAngle(Outtake.Angle.INTAKE)),
                new SleepAction(0.2),
                new InstantAction(()-> claw.updateState(Claw.ClawState.CLOSED,ClawSide.BOTH)),
                new SleepAction(0.1),
                new InstantAction(()->intake.updateClawState(Intake.ClawState.OPEN,ClawSide.BOTH))


                );
    }

    SequentialAction IntakePixels32()
    {
        return new SequentialAction(
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                intakeActions.openExtension(PIXEL_EXTENSION),
                new SleepAction(0.2),
                intakeActions.moveIntake(Intake.Angle.TOP_32),
                intakeActions.moveIntakeClaw(Intake.ClawState.CLOSE, ClawSide.BOTH),
                new SleepAction(1),
                new InstantAction(() -> intakeExtension.setAggresive(false)),
                intakeActions.closeExtension(),
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                new InstantAction(()-> depositActions.moveOuttake(Outtake.Angle.INTAKE)),
                new SleepAction(0.2),
                new InstantAction(()->depositActions.moveClaw(Claw.ClawState.CLOSED,ClawSide.BOTH)),
                new SleepAction(0.1),
                new InstantAction(()->intakeActions.moveIntakeClaw(Intake.ClawState.OPEN,ClawSide.BOTH))

        );
    }

    SequentialAction IntakePixels1()
    {
        return new SequentialAction(
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                intakeActions.openExtension(PIXEL_EXTENSION),
                new SleepAction(0.2),
                intakeActions.moveIntake(Intake.Angle.INTAKE),
                intakeActions.moveIntakeClaw(Intake.ClawState.CLOSE, ClawSide.BOTH),
                new SleepAction(1),
                new InstantAction(() -> intakeExtension.setAggresive(false)),
                intakeActions.closeExtension(),
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                new InstantAction(()-> depositActions.moveOuttake(Outtake.Angle.INTAKE)),
                new SleepAction(0.2),
                new InstantAction(()->depositActions.moveClaw(Claw.ClawState.CLOSED,ClawSide.BOTH)),
                new SleepAction(0.1),
                new InstantAction(()->intakeActions.moveIntakeClaw(Intake.ClawState.OPEN,ClawSide.BOTH))

        );
    }


    SequentialAction prepOuttake()
    {
        return new SequentialAction(
                depositActions.moveOuttake(Outtake.Angle.OUTTAKE),
                depositActions.moveElevator(1000)
        );
    }


}