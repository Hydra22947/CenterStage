package org.firstinspires.ftc.teamcode.auto.OldAuto.BasicAutos;

// RR-specific imports

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
import org.firstinspires.ftc.teamcode.auto.Actions.IntakeActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoSettings;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineRedRight;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
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
    IntakeActions intakeActions;
    UpdateActions updateActions;

    public static AutoSettings.PropLocation propLocation = AutoSettings.PropLocation.MIDDLE;
    PropPipelineRedRight propPipelineRedRight;
    OpenCvWebcam webcam;

    public int tempHeight = 900;
    boolean first = true;

    @Override
    public void runOpMode() {
        BetterGamepad betterGamepad2 = new BetterGamepad(gamepad2);
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        propPipelineRedRight = new PropPipelineRedRight();
        robot.init(hardwareMap, telemetry, autoConstants.startPoseRedRight);

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
        intakeActions = new IntakeActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);


        SequentialAction placePurplePixelBlueLeft = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(.5),
                intakeActions.openExtension(600),
                new SleepAction(1.5),
                intakeActions.release(IntakeActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
                intakeActions.openExtension(550),
                new SleepAction(0.2),
                intakeActions.moveIntake(Intake.Angle.MID),
                intakeActions.closeExtension(),
                intakeActions.lock(IntakeActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction retractDepositBlueLeft = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction placePurplePixelBlueMiddle = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(0.5),
                intakeActions.openExtension(790),
                new SleepAction(1.5),
                intakeActions.release(IntakeActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
                intakeActions.closeExtension(),
                new SleepAction(0.1),
                intakeActions.moveIntake(Intake.Angle.MID),
                intakeActions.lock(IntakeActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction retractDepositBlueMiddle = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction depositBlueRight = new SequentialAction(
                depositActions.readyForDeposit(tempHeight),
                //depositActions.placePixel(),
                new SleepAction(0.5),
                depositActions.moveElevator(1500)
        );

        SequentialAction placePurplePixelBlueRight = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(0.5),
                intakeActions.openExtension(800),
                new SleepAction(1),
                intakeActions.release(IntakeActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.1),
                intakeActions.openExtension(800),
                new SleepAction(0.2),
                intakeActions.moveIntake(Intake.Angle.MID),
                intakeActions.closeExtension(),
                intakeActions.lock(IntakeActions.CloseClaw.BOTH_CLOSE)
        );
        SequentialAction retractDepositBlueRight = new SequentialAction(
                depositActions.retractDeposit()
        );

        Action trajRedRight =
                robot.drive.actionBuilder(robot.drive.pose)
                        .stopAndAdd(depositActions.readyForDeposit(1050))
                        .splineToLinearHeading(new Pose2d(48, -34.25, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixelBlueLeft)
                        .setTangent(0)
                        .waitSeconds(.2)
                        .stopAndAdd(intakeActions.moveIntake(Intake.Angle.MID))
                        .waitSeconds(.2)
                        .stopAndAdd(intakeActions.closeExtension())
                        .strafeTo(new Vector2d(50.75, -40.5))
                        .stopAndAdd(depositBlueRight)
                        .waitSeconds(0.5)
                        //Park
                        .setTangent(Math.toRadians(-90))
                        .strafeTo(new Vector2d(47, -40.5))
                        .strafeTo(new Vector2d(45, -60))
                        .stopAndAdd(retractDepositBlueLeft)
                        .turnTo(Math.toRadians(90))
                        .build();

        //Park Close To Backdrop
                        /*
                        .strafeTo(new Vector2d(52,15))
                        .stopAndAdd(retractDepositBlueMiddle)
                        .turnTo(Math.toRadians(-90))
                        .build();
                         */

        Action trajRedMiddle =
                robot.drive.actionBuilder(robot.drive.pose)
                        .stopAndAdd(depositActions.readyForDeposit(tempHeight))
                        .splineToLinearHeading(new Pose2d(40, -26, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixelBlueMiddle)
                        .setTangent(0)
                        .waitSeconds(.2)
                        .stopAndAdd(intakeActions.moveIntake(Intake.Angle.MID))
                        .waitSeconds(.2)
                        .stopAndAdd(intakeActions.closeExtension())
                        //Place Preload on board
                        .splineToLinearHeading(new Pose2d(51, -34, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(.1)
                        .stopAndAdd(depositBlueRight)
                        .waitSeconds(0.5)
                        //Park
                        .setTangent(Math.toRadians(-90))
                        .strafeTo(new Vector2d(45, -33))
                        //Park Close To Wall
                        .strafeTo(new Vector2d(45, -60))
                        .stopAndAdd(retractDepositBlueMiddle)
                        .turnTo(Math.toRadians(90))
                        .build();

        //Park Close To Backdrop
                        /*
                        .strafeTo(new Vector2d(52,15))
                        .stopAndAdd(retractDepositBlueMiddle)
                        .turnTo(Math.toRadians(-90))
                        .build();
                         */


        Action trajRedLeft =
                robot.drive.actionBuilder(robot.drive.pose)
                        .stopAndAdd(depositActions.readyForDeposit(tempHeight))
                        .splineToLinearHeading(new Pose2d(31, -32, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixelBlueRight)
                        .setTangent(0)
                        .waitSeconds(.2)
                        .stopAndAdd(intakeActions.moveIntake(Intake.Angle.MID))
                        .waitSeconds(.2)
                        .stopAndAdd(intakeActions.closeExtension())
                        //Place Preload on board
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(51, -28))
                        .stopAndAdd(depositBlueRight)
                        .waitSeconds(0.3)
                        //Park
                        .setTangent(Math.toRadians(-90))
                        .strafeTo(new Vector2d(48, -28))
                        .stopAndAdd(retractDepositBlueRight)
                        .strafeToLinearHeading(new Vector2d(45, -55), Math.toRadians(90))
                        .strafeTo(new Vector2d(45.25, -60))
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
            telemetry.addData("POS", propLocation.name());

            switch (propPipelineRedRight.getLocation()) {
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

            if(betterGamepad2.dpadUpOnce())
            {
                if(first)
                {
                    webcam.stopStreaming();
                    first = false;
                }

                propLocation = cycleVision(propLocation);
            }
            else if(betterGamepad2.dpadDownOnce())
            {
                initCamera();
                webcam.setPipeline(propPipelineRedRight);
            }


            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addLine("Initialized");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        webcam.stopStreaming();

        switch (propLocation) {
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