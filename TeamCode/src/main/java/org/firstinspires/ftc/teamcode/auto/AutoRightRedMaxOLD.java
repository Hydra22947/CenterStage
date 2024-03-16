package org.firstinspires.ftc.teamcode.auto;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "2+2 - Auto Red Right OLD")
@Disabled
public class AutoRightRedMaxOLD extends LinearOpMode {
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

    public static int tempHeight = 1100;
    public enum PropLocation
    {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static PropLocation propLocation = PropLocation.MIDDLE;
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
        intakeActions = new PlacePurpleActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);

        SequentialAction depositRedLeft = new SequentialAction(
                depositActions.readyForDeposit(tempHeight + 200),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD ,600)
        );

        SequentialAction placePurplePixelRedLeft = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(0.5),
                intakeActions.openExtension(1080),
                new SleepAction(0.6),
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
                intakeActions.openExtension(800),
                new SleepAction(0.2),
                intakeActions.moveIntake(Intake.Angle.MID),
                intakeActions.closeExtension(),
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction retractDepositRedLeft = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction depositRedMiddle = new SequentialAction(
                depositActions.readyForDeposit(tempHeight + 150),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD ,600),
                new SleepAction(0.5),
                depositActions.retractDeposit()
        );

        SequentialAction placePurplePixelRedMiddle = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(0.5),
                intakeActions.openExtension(790),
                new SleepAction(1),
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.75),
                intakeActions.moveIntake(Intake.Angle.MID),
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                intakeActions.closeExtension()
        );

        SequentialAction retractDepositRedMiddle = new SequentialAction(
                depositActions.retractDeposit()
        );


        SequentialAction depositRedRight = new SequentialAction(
                depositActions.readyForDeposit(tempHeight + 75),
                new SleepAction(1),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(1),
                depositActions.retractDeposit()
        );

        SequentialAction depositSecondCycle = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),

                intakeActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(1),
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),

                new SleepAction(0.1),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),

                new SleepAction(0.5),
                depositActions.moveElevator(1100),
                depositActions.retractDeposit()
        );

        SequentialAction placePurplePixelRedRight = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(.5),
                intakeActions.openExtension(640),
                new SleepAction(1),
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
                intakeActions.openExtension(550),
                new SleepAction(0.2),
                intakeActions.moveIntake(Intake.Angle.MID),
                intakeActions.closeExtension(),
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction transfer = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.75),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );
        SequentialAction retractDepositRedRight = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction readyForDeposit = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),
                depositActions.readyForDeposit(tempHeight)

        );


        SequentialAction openIntakeWhitePixel = new SequentialAction(
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                intakeActions.openExtension(1000)
        );

        SequentialAction closeIntakeWhitePixel = new SequentialAction(
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                intakeActions.moveStack(),
                new SleepAction(.5),
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                intakeActions.closeExtension()
        );


        Action trajRedLeft =
                robot.drive.actionBuilder(robot.drive.pose)
                        .stopAndAdd(depositActions.readyForDeposit(1050))
                        .splineToLinearHeading(new Pose2d(34.7, -30.25, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixelRedLeft)
                        .setTangent(0)
                        //Place Preload on board
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(50.75, -26.8))
                        .stopAndAdd(depositRedLeft)
                        .waitSeconds(0.5)

                        //Intake 54
                        .waitSeconds(0.2)
                        .setTangent(Math.toRadians(120))
                        .stopAndAdd(intakeActions.moveIntake(Intake.Angle.TOP_54))
                        .splineToConstantHeading(new Vector2d(30, -8), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-32, -12, Math.toRadians(0)), Math.toRadians(180))

                        .stopAndAdd(openIntakeWhitePixel)
                        .waitSeconds(.2)
                        .strafeToLinearHeading(new Vector2d(-34, -14), Math.toRadians(0))

                        .stopAndAdd(closeIntakeWhitePixel)
                        .waitSeconds(0.5)
                        .stopAndAdd(transfer)

                        //Deposit
                        .strafeToLinearHeading(new Vector2d(30, -9), Math.toRadians(0))
                        .stopAndAdd(readyForDeposit)
                        .splineToLinearHeading(new Pose2d(50.5, -40, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(depositSecondCycle)
                        .waitSeconds(0.5)


                        //Park
                        .setTangent(Math.toRadians(90))
                        .strafeTo(new Vector2d(48, -27.4))
                        .stopAndAdd(retractDepositRedLeft)
                        .strafeToLinearHeading(new Vector2d(45, -55),Math.toRadians(90))
                        .build();

        // near the other back drop
        //.strafeToLinearHeading(new Vector2d(52, -15),Math.toRadians(90))
        // .build();

        Action trajRedMiddle =
                robot.drive.actionBuilder(robot.drive.pose)
                        .stopAndAdd(depositActions.readyForDeposit(1000))
                        .splineToLinearHeading(new Pose2d(41 , -25, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixelRedMiddle)
                        .setTangent(0)
                        //Place Preload on board
                        .splineToLinearHeading(new Pose2d(51.5, -34, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(.1)
                        .stopAndAdd(depositRedMiddle)
                        .waitSeconds(0.5)

                        //Intake 54
                        .waitSeconds(0.2)
                        .setTangent(Math.toRadians(120))
                        .stopAndAdd(intakeActions.moveIntake(Intake.Angle.TOP_54))
                        .splineToConstantHeading(new Vector2d(30, -8), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-32, -12, Math.toRadians(0)), Math.toRadians(180))

                        .stopAndAdd(openIntakeWhitePixel)
                        .waitSeconds(.2)
                        .strafeToLinearHeading(new Vector2d(-34, -14), Math.toRadians(0))

                        .stopAndAdd(closeIntakeWhitePixel)
                        .waitSeconds(0.5)
                        .stopAndAdd(transfer)

                        //Deposit
                        .strafeToLinearHeading(new Vector2d(30, -9), Math.toRadians(0))
                        .stopAndAdd(readyForDeposit)
                        .splineToLinearHeading(new Pose2d(50.5, -40, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(depositSecondCycle)
                        .waitSeconds(0.5)




                        //Park
                        .setTangent(Math.toRadians(90))
                        .strafeTo(new Vector2d(48, -35.6))
                        .stopAndAdd(retractDepositRedMiddle)
                        .strafeToLinearHeading(new Vector2d(45, -55),Math.toRadians(90))

                        .build();

        // near the other back drop
        //.strafeToLinearHeading(new Vector2d(52, -15),Math.toRadians(90))
        // .build();

        Action trajRedRight =
                robot.drive.actionBuilder(robot.drive.pose)
                        //Place purple pixel
                        .stopAndAdd(depositActions.readyForDeposit(1000))
                        .splineToLinearHeading(new Pose2d(48.25, -29.5, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixelRedRight)

                        //Place preload on board
                        .setTangent(0)
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(50.5, -40))
                        .stopAndAdd(depositRedRight)
                        .waitSeconds(0.5)

                        //Intake 54
                        .waitSeconds(0.2)
                        .setTangent(Math.toRadians(120))
                        .stopAndAdd(intakeActions.moveIntake(Intake.Angle.TOP_54))
                        .splineToConstantHeading(new Vector2d(30, -8), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-32, -12, Math.toRadians(0)), Math.toRadians(180))

                        .stopAndAdd(openIntakeWhitePixel)
                        .waitSeconds(.2)
                        .strafeToLinearHeading(new Vector2d(-34, -14), Math.toRadians(0))

                        .stopAndAdd(closeIntakeWhitePixel)
                        .waitSeconds(0.5)
                        .stopAndAdd(transfer)

                        //Deposit
                        .strafeToLinearHeading(new Vector2d(30, -9), Math.toRadians(0))
                        .stopAndAdd(readyForDeposit)
                        .splineToLinearHeading(new Pose2d(50.5, -40, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(depositSecondCycle)
                        .waitSeconds(0.5)

                        //Park
                        .setTangent(Math.toRadians(90))
                        .strafeTo(new Vector2d(48, -42))
                        .stopAndAdd(retractDepositRedRight)
                        .strafeToLinearHeading(new Vector2d(45, -55),Math.toRadians(90))
                        .build();

        // near the other back drop
        //.strafeToLinearHeading(new Vector2d(52, -15),Math.toRadians(90))
        // .build();


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