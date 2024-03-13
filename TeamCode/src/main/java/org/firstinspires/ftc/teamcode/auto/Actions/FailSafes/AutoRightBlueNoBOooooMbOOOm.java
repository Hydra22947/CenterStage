package org.firstinspires.ftc.teamcode.auto.Actions.FailSafes;

// RR-specific imports

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static org.firstinspires.ftc.teamcode.auto.AutoSettings.writeToFile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineBlueRight;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

@Config
@Autonomous(name = "AP Test Auto Blue Right")
public class AutoRightBlueNoBOooooMbOOOm extends LinearOpMode {
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


    PropPipelineBlueRight propPipelineBlueRight;

    @Override
    public void runOpMode() {
        BetterGamepad betterGamepad2 = new BetterGamepad(gamepad2);
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        propPipelineBlueRight = new PropPipelineBlueRight();
        robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueRight);

        autoConstants = new AutoConstants();

        elevator = new Elevator(true);
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension(true);

        intakeExtension.setAuto(true);
        elevator.setAuto(true);

        depositActions = new DepositActions(elevator, intake, claw, outtake, intakeExtension);
        placePurpleActions = new PlacePurpleActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);


        SequentialAction readyIntakeBlue = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.MID)
        );


        SequentialAction depositBlueMiddle = new SequentialAction(

                placePurpleActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(1),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),
                new SleepAction(0.5),
                depositActions.moveElevator(1500)
        );

        SequentialAction transferBlueMiddle = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                placePurpleActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                placePurpleActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(1),
                placePurpleActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );


        SequentialAction retractDepositBlueMiddle = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction intakePixelBlueLeft = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                placePurpleActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH)

        );

        SequentialAction readyForDepositHigh = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.MID),
                new SleepAction(.25),
                depositActions.readyForDeposit(1500)
        );

        Action trajBlueMiddle =
                robot.drive.actionBuilder(robot.drive.pose)
                        //place purple
                        .strafeToLinearHeading(new Vector2d(-34.5, 34), Math.toRadians(-90))

                        .strafeToLinearHeading(new Vector2d(-49.5, 24), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(-53.5, 26), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(-44.25, 10), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(20, 8, Math.toRadians(0)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(32, 32), Math.toRadians(90))
                        .waitSeconds(1)
                        .stopAndAdd(new CheckAprilTagAction(
                                new InstantAction(() -> robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))),
                                robot.drive.actionBuilder(new Pose2d(32,32,Math.toRadians(0)))
                                        .splineToConstantHeading(new Vector2d(52, 32), Math.toRadians(-10)).build()
                                ))

                        // detect

                        .waitSeconds(.5)
                        .setTangent(Math.toRadians(90))
                        //Park - Close to other board
                        .strafeToLinearHeading(new Vector2d(46, 32), Math.toRadians(-90))
                        .build();

        while (opModeInInit() && !isStopRequested()) {
            betterGamepad2.update();
            intake.setAngle(Intake.Angle.MID);
            intakeExtension.setTarget(0);
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addData("POS", propPipelineBlueRight.getLocation());
            telemetry.addData("NO PROP", propPipelineBlueRight.NO_PROP);

            telemetry.addLine("Initialized");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;
        robot.drive.startAutoTimer();

        runBlocking(new ParallelAction(
                trajBlueMiddle,
                updateActions.updateSystems()
        ));

        while (opModeIsActive()) {
            robot.drive.updatePoseEstimate();
        }

        writeToFile(robot.drive.pose.heading.log());
    }

}