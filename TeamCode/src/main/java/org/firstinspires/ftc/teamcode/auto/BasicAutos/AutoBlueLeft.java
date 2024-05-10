package org.firstinspires.ftc.teamcode.auto.BasicAutos;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import static org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoSettings.cycleVision;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
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
import org.firstinspires.ftc.teamcode.auto.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoSettings;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineBlueLeft;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "2+0 - Auto Blue Left")
public class AutoBlueLeft extends LinearOpMode {
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

    public static AutoSettings.PropLocation propLocation = AutoSettings.PropLocation.MIDDLE;
    PropPipelineBlueLeft propPipelineBlueLeft;
    OpenCvWebcam webcam;
    boolean vision = true;

    public static int tempHeight = 900;
    boolean first = true;

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


        elevator = new LiftSubsystem(true);
        outtake = new OuttakeSubsystem();
        claw = new Claw();
        intake = new IntakeSubsystem();
        intakeExtension = new IntakeExtensionSubsystem(true);

        intakeExtension.setAuto(true);
        elevator.setAuto(true);

        depositActions = new DepositActions(elevator, intake, claw, outtake , intakeExtension);
        placePurpleActions = new PlacePurpleActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);

        SequentialAction depositBlueLeft = new SequentialAction(
                depositActions.readyForDeposit(tempHeight),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD ,600),
                new SleepAction(0.5),
                depositActions.moveElevator(tempHeight + 400)
        );

        SequentialAction placePurplePixelBlueLeft = new SequentialAction(
                placePurpleActions.moveIntake(IntakeSubsystem.Angle.INTAKE),
                new SleepAction(.5),
                placePurpleActions.openExtension(615),
                new SleepAction(1.5),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
               placePurpleActions.openExtension(550),
                new SleepAction(0.2),
                placePurpleActions.moveIntake(IntakeSubsystem.Angle.MID),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction retractDepositBlueLeft = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction depositBlueMiddle = new SequentialAction(

                depositActions.placePixel(DepositActions.Cycles.PRELOAD ,600),
                new SleepAction(0.5),
                depositActions.readyForDeposit(tempHeight + 150),
                depositActions.retractDeposit()
        );

        SequentialAction placePurplePixelBlueMiddle = new SequentialAction(
                placePurpleActions.moveIntake(IntakeSubsystem.Angle.INTAKE),
                new SleepAction(0.5),
                placePurpleActions.openExtension(790),
                new SleepAction(1.5),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
                placePurpleActions.closeExtension(),
                new SleepAction(0.1),
                placePurpleActions.moveIntake(IntakeSubsystem.Angle.MID),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction retractDepositBlueMiddle = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction depositBlueRight = new SequentialAction(
                depositActions.readyForDeposit(tempHeight),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD ,600),
                new SleepAction(0.5),
                depositActions.moveElevator(tempHeight + 300)
        );

        SequentialAction placePurplePixelBlueRight = new SequentialAction(
                placePurpleActions.moveIntake(IntakeSubsystem.Angle.INTAKE),
                new SleepAction(0.5),
                placePurpleActions.openExtension(800),
                new SleepAction(1),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
                placePurpleActions.openExtension(780),
                new SleepAction(0.2),
                placePurpleActions.moveIntake(IntakeSubsystem.Angle.MID),
                placePurpleActions.closeExtension(),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );
        SequentialAction retractDepositBlueRight = new SequentialAction(
                depositActions.retractDeposit()
        );

        Action trajBlueLeft =
                robot.drive.actionBuilder(robot.drive.pose)
                        .stopAndAdd(depositActions.readyForDeposit(1050))
                        .splineToLinearHeading(new Pose2d(48, 34.25, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixelBlueLeft)
                        .setTangent(0)
                        .waitSeconds(.1)
                        .stopAndAdd(placePurpleActions.moveIntake(IntakeSubsystem.Angle.MID))
                        .waitSeconds(.2)
                        .stopAndAdd(placePurpleActions.closeExtension())
                        .strafeTo(new Vector2d(50, 40.7))
                        .stopAndAdd(depositBlueLeft)
                        .waitSeconds(0.5)
                        //Park
                        .setTangent(Math.toRadians(90))
                        .strafeTo(new Vector2d(47, 40.5))
                        .strafeTo(new Vector2d(45, 58))
                        .stopAndAdd(retractDepositBlueLeft)
                        .turnTo(Math.toRadians(-90))
                        .build();

                        //Park Close To Backdrop
                        /*
                        .strafeTo(new Vector2d(52,15))
                        .stopAndAdd(retractDepositBlueMiddle)
                        .turnTo(Math.toRadians(-90))
                        .build();
                         */

        Action trajBlueMiddle =
                robot.drive.actionBuilder(robot.drive.pose)
                        .stopAndAdd(depositActions.readyForDeposit(tempHeight))
                        .splineToLinearHeading(new Pose2d(40 , 26, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixelBlueMiddle)
                        .setTangent(0)
                        .waitSeconds(.2)
                        .stopAndAdd(placePurpleActions.moveIntake(IntakeSubsystem.Angle.MID))
                        .waitSeconds(.2)
                        .stopAndAdd(placePurpleActions.openExtension(0))
                        //Place Preload on board
                        .splineToLinearHeading(new Pose2d(51, 34, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(.1)
                        .stopAndAdd(depositBlueMiddle)
                        .waitSeconds(0.5)
                        //Park
                        .setTangent(Math.toRadians(90))
                        .strafeTo(new Vector2d(50, 33))
                        //Park Close To Wall
                        .strafeTo(new Vector2d(45, 60))
                        .stopAndAdd(retractDepositBlueMiddle)
                        .turnTo(Math.toRadians(-90))
                        .build();

                        //Park Close To Backdrop
                        /*
                        .strafeTo(new Vector2d(52,15))
                        .stopAndAdd(retractDepositBlueMiddle)
                        .turnTo(Math.toRadians(-90))
                        .build();
                         */


        Action trajBlueRight =
                robot.drive.actionBuilder(robot.drive.pose)
                        .stopAndAdd(depositActions.readyForDeposit(tempHeight))
                        .splineToLinearHeading(new Pose2d(31,  32, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixelBlueRight)
                        .setTangent(0)
                        .waitSeconds(.2)
                        .stopAndAdd(placePurpleActions.closeExtension())
                        //Place Preload on board
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(51, 28))
                        .stopAndAdd(depositBlueRight)
                        .waitSeconds(0.5)
                        //Park
                        .setTangent(Math.toRadians(90))
                        .strafeTo(new Vector2d(48, 28.5))
                        .stopAndAdd(retractDepositBlueRight)
                        .strafeToLinearHeading(new Vector2d(45, 55), Math.toRadians(-90))
                        .strafeTo(new Vector2d(45.25, 60))
                        .build();


                        //Park Close To Backdrop
                        /*
                        .strafeTo(new Vector2d(52,15))
                        .stopAndAdd(retractDepositBlueMiddle)
                        .turnTo(Math.toRadians(-90))
                        .build();
                         */

        while (opModeInInit() && !isStopRequested()) {

            betterGamepad2.update();

            intake.setAngle(IntakeSubsystem.Angle.MID);

            intake.updateState(IntakeSubsystem.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(OuttakeSubsystem.Angle.INTAKE);
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
                        trajBlueLeft,
                        updateActions.updateSystems()
                ));
                break;
            case RIGHT:
                runBlocking(new ParallelAction(
                        trajBlueRight,
                        updateActions.updateSystems()
                ));
                break;
            case MIDDLE:
                runBlocking(new ParallelAction(
                        trajBlueMiddle,
                        updateActions.updateSystems()
                ));
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



}