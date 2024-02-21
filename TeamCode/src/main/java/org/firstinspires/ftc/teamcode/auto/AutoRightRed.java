package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static org.firstinspires.ftc.teamcode.auto.AutoSettings.writeToFile;

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
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.testing.harman.PropPipelineRedRight;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "2+0 - Auto Red Right")
public class AutoRightRed extends LinearOpMode {
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

    enum PropLocation
    {
        LEFT,
        MIDDLE,
        RIGHT
    }

    PropLocation propLocation = PropLocation.MIDDLE;
    PropPipelineRedRight propPipelineRedRight;
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, autoConstants.startPoseRedRight);

        propPipelineRedRight = new PropPipelineRedRight();
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

        depositActions = new DepositActions(elevator, intake, claw, outtake , intakeExtension);
        placePurpleActions = new PlacePurpleActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);

        SequentialAction depositRedLeft = new SequentialAction(
                depositActions.readyForDeposit(950),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD ,600),
                new SleepAction(0.5),
                depositActions.retractDeposit()
        );

        SequentialAction placePurplePixelRedLeft = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(0.5),
                placePurpleActions.openExtension(1030),
                new SleepAction(1.45),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
                placePurpleActions.moveIntake(Intake.Angle.MID),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction retractDepositRedLeft = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction depositRedMiddle = new SequentialAction(
                depositActions.readyForDeposit(950),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD ,600),
                new SleepAction(0.5),
                depositActions.retractDeposit()
        );

        SequentialAction placePurplePixelRedMiddle = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(0.5),
                placePurpleActions.openExtension(820),
                new SleepAction(1),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.75),
                placePurpleActions.moveIntake(Intake.Angle.MID),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction retractDepositRedMiddle = new SequentialAction(
                depositActions.retractDeposit()
        );


        SequentialAction depositRedRight = new SequentialAction(
                depositActions.readyForDeposit(950),
                new SleepAction(1),
                placePurpleActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(1),
                depositActions.retractDeposit()
        );

        SequentialAction placePurplePixelRedRight = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(.5),
                placePurpleActions.openExtension(640),
                new SleepAction(1),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
                placePurpleActions.moveIntake(Intake.Angle.MID),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction retractDepositRedRight = new SequentialAction(
                depositActions.retractDeposit()
        );

        Action trajRedLeft =
                robot.drive.actionBuilder(robot.drive.pose)
                        .stopAndAdd(depositActions.readyForDeposit(950))
                        .splineToLinearHeading(new Pose2d(34.7, -30.25, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixelRedLeft)
                        .setTangent(0)
                        .stopAndAdd(placePurpleActions.closeExtension())
                        //Place Preload on board
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(50.75, -27.5))
                        .stopAndAdd(depositRedLeft)
                        .waitSeconds(0.5)
                        //Park
                        .setTangent(Math.toRadians(90))
                        .stopAndAdd(retractDepositRedLeft)
                        .strafeTo(new Vector2d(45, -60))
                        .turnTo(Math.toRadians(90))
                        .build();

        Action trajRedMiddle =
                robot.drive.actionBuilder(robot.drive.pose)
                        .stopAndAdd(depositActions.readyForDeposit(950))
                        .splineToLinearHeading(new Pose2d(41 , -25, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixelRedMiddle)
                        .setTangent(0)
                        .stopAndAdd(placePurpleActions.closeExtension())
                        //Place Preload on board
                        .splineToLinearHeading(new Pose2d(51.5, -37.2, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(.1)
                        .stopAndAdd(depositRedMiddle)
                        .waitSeconds(0.5)
                        //Park
                        .setTangent(Math.toRadians(90))
                        .stopAndAdd(retractDepositRedMiddle)
                        .strafeTo(new Vector2d(45, -60))
                        .turnTo(Math.toRadians(90))

                        .build();

        Action trajRedRight =
                robot.drive.actionBuilder(robot.drive.pose)
                        .stopAndAdd(depositActions.readyForDeposit(950))
                        .splineToLinearHeading(new Pose2d(48.25, -31.25, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixelRedRight)
                        .setTangent(0)
                        .stopAndAdd(placePurpleActions.closeExtension())
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(50.5, -42.5))
                        .stopAndAdd(depositRedRight)
                        .waitSeconds(0.5)
                        //Park
                        .setTangent(Math.toRadians(90))
                        .stopAndAdd(retractDepositRedRight)
                        .strafeTo(new Vector2d(45, -60))
                        .turnTo(Math.toRadians(90))
                        .build();

        while (opModeInInit() && !isStopRequested()) {
            intake.setAngle(Intake.Angle.MID);
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            telemetry.addData("POS", propPipelineRedRight.getLocation());

            switch (propPipelineRedRight.getLocation())
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
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addLine("Initialized");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        switch (propLocation)
        {
            case RIGHT:
                runBlocking(new ParallelAction(
                        trajRedRight,
                        updateActions.updateSystems()
                ));
                break;
            case MIDDLE:
                runBlocking(new ParallelAction(
                        trajRedMiddle,
                        updateActions.updateSystems()
                ));
                break;
            case LEFT:
                runBlocking(new ParallelAction(
                        trajRedLeft,
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