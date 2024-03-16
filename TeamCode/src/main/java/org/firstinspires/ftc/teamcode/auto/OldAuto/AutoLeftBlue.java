package org.firstinspires.ftc.teamcode.auto.OldAuto;

// RR-specific imports

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import static org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoSettings.writeToFile;

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
@Autonomous(name = "2+0 - Auto Blue Left")
public class AutoLeftBlue extends LinearOpMode {
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

    public enum PropLocation
    {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static PropLocation propLocation = PropLocation.MIDDLE;
    PropPipelineBlueLeft propPipelineBlueLeft;
    OpenCvWebcam webcam;

    public static int tempHeight = 900;

    @Override
    public void runOpMode() {
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
        placePurpleActions = new PlacePurpleActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);

        SequentialAction depositBlueLeft = new SequentialAction(
                depositActions.readyForDeposit(tempHeight),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD ,600),
                new SleepAction(0.5),
                depositActions.moveElevator(tempHeight + 400)
        );

        SequentialAction placePurplePixelBlueLeft = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(.5),
                placePurpleActions.openExtension(615),
                new SleepAction(1.5),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
               placePurpleActions.openExtension(550),
                new SleepAction(0.2),
                placePurpleActions.moveIntake(Intake.Angle.MID),
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
                placePurpleActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(0.5),
                placePurpleActions.openExtension(790),
                new SleepAction(1.5),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
                placePurpleActions.closeExtension(),
                new SleepAction(0.1),
                placePurpleActions.moveIntake(Intake.Angle.MID),
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
                placePurpleActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(0.5),
                placePurpleActions.openExtension(820),
                new SleepAction(0.6),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
                placePurpleActions.openExtension(780),
                new SleepAction(0.2),
                placePurpleActions.moveIntake(Intake.Angle.MID),
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
                        .stopAndAdd(placePurpleActions.moveIntake(Intake.Angle.MID))
                        .waitSeconds(.2)
                        .stopAndAdd(placePurpleActions.closeExtension())
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
                        .splineToLinearHeading(new Pose2d(34.7,  32, Math.toRadians(0)), Math.toRadians(0))
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

            intake.setAngle(Intake.Angle.MID);

            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addData("POS", propPipelineBlueLeft.getLocation());
            telemetry.addData("NO PROP", propPipelineBlueLeft.NO_PROP);
            switch (propPipelineBlueLeft.getLocation())
            {
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

            if(betterGamepad2.dpadUpOnce())
            {
                propPipelineBlueLeft.NO_PROP++;
            }
            else if(betterGamepad2.dpadDownOnce())
            {
                propPipelineBlueLeft.NO_PROP--;
            }
            telemetry.addLine("Initialized");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

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


        while (opModeIsActive())
        {
            robot.drive.updatePoseEstimate();
        }

        writeToFile(robot.drive.pose.heading.log());
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