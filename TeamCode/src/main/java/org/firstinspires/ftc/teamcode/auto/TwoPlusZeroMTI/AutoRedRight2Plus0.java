package org.firstinspires.ftc.teamcode.auto.TwoPlusZeroMTI;

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
@Autonomous(name = "2+0 MTI RedRight" , group = "AutoRed")
public class AutoRedRight2Plus0 extends LinearOpMode {
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

    public static int PIXEL_EXTENSION = 200;

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
        robot.init(hardwareMap, telemetry, autoConstants.startPoseRedRight);

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
        intakeActions = new IntakeActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);


        SequentialAction placePurplePixelAction_Middle = new SequentialAction(
                new ParallelAction(
                        intakeActions.moveIntake(Intake.Angle.INTAKE),
                        new SleepAction(0.5)),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH)
        );

        SequentialAction placePurplePixelAction_Right = new SequentialAction(
                new ParallelAction(
                        intakeActions.moveIntake(Intake.Angle.INTAKE),
                        new SleepAction(0.6)),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN,ClawSide.BOTH)
        );


        SequentialAction placePurplePixelAction_Left = new SequentialAction(
                new ParallelAction(
                        intakeActions.moveIntake(Intake.Angle.INTAKE),
                        new SleepAction(0.3)),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH)
        );

        SequentialAction retractPurpleAction = new SequentialAction(
                new ParallelAction(
                        new InstantAction(() -> intakeExtension.setAggresive(true)),
                        new InstantAction(() -> intakeActions.closeExtension()),
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



        Action trajRedRight =
                robot.drive.actionBuilder(robot.drive.pose)
                        .afterTime(1.8, placePurplePixelSequence_Right)
                        .splineToSplineHeading(new Pose2d(15, -40, Math.toRadians(0)), Math.toRadians(90))
                        .afterTime(0, placeYellowPixel)
                        .splineToLinearHeading(new Pose2d(45, -40,Math.toRadians(0)), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(45,-60), Math.toRadians(270))

                        .build();


        Action trajRedMiddle =
                robot.drive.actionBuilder(robot.drive.pose)
                        .afterTime(1.6, placePurplePixelSequence_Middle)
                        .splineToLinearHeading(new Pose2d(20, -27, Math.toRadians(0)), Math.toRadians(90))
                        .afterTime(0.8, placeYellowPixel)
                        .splineToSplineHeading(new Pose2d(45, -35,Math.toRadians(0)), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(45,-60), Math.toRadians(270))



                        .build();


        Action trajRedLeft =
                robot.drive.actionBuilder(robot.drive.pose)

                        .afterTime(1, placePurplePixelSequence_Left)
                        .splineToLinearHeading(new Pose2d(17, -35, Math.toRadians(0)), Math.toRadians(90))
                        .afterTime(1.1, placeYellowPixel)
                        .splineToSplineHeading(new Pose2d(48, -32, Math.toRadians(0)), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(45,-60), Math.toRadians(270))

                        .build();





        runMiddle = new SequentialAction(trajRedMiddle);
        runRight = new SequentialAction(trajRedRight);
        runLeft = new SequentialAction(trajRedLeft);

        while (opModeInInit() && !isStopRequested()) {

            betterGamepad2.update();

            intake.setAngle(Intake.Angle.AUTO_MID);

            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addData("POS", propLocation.name());


            if (vision) {
                switch (propPipelineBlueLeft.getLocation()) {
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
                        runLeft,
                        updateActions.updateSystems()));
                break;
            case RIGHT:
                runBlocking(new ParallelAction(
                        runRight,
                        updateActions.updateSystems()));
                break;
            case MIDDLE:
                runBlocking(new ParallelAction(
                        runMiddle,
                        updateActions.updateSystems()));
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



}



