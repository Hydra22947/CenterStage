package org.firstinspires.ftc.teamcode.auto.machines;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.old_with_cycles.AutoConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@Autonomous(group = "advanced")
public class AsyncFollowingFSM extends LinearOpMode {

    RobotHardware robot = RobotHardware.getInstance();
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;
    SampleMecanumDrive drive;

    public static int elevatorTarget = 950;
    public static int intakeTarget = 950;
    public static int intakeResetDelay = 1500, resetDelay = 500, releasePixelsDelay = 2500;

    double intakeReleaseTime = 0, resetTime = 0, releaseTime = 0;
    boolean runStateMachine = false, donePlacinng = false;
    enum State {
        DRIVE_TO_BOARD,
        PLACE_DOUBLE_PIXEL,
        PARK,
        IDLE
    }

    enum SystemsState
    {
        IDLE,
        OPEN_SYSTEMS,
        RELEASE_PIXELS,
        RESET
    }

    SystemsState systemsState = SystemsState.IDLE;

    State currentState = State.IDLE;
    ElapsedTime timer;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry, true);

        timer = new ElapsedTime();
        elevator = new Elevator(true);
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension(true);
        drive = new SampleMecanumDrive(hardwareMap);
        elevator.setFirstPID(false);

        // Set inital pose
        drive.setPoseEstimate(AutoConstants.startPoseBlueLeft);

        // Let's define our trajectories
        Trajectory goToBoard = drive.trajectoryBuilder(AutoConstants.startPoseBlueLeft)
                .splineToLinearHeading(new Pose2d(52.5, 33, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory park = drive.trajectoryBuilder(goToBoard.end())
                .lineToLinearHeading(new Pose2d(45, 0, Math.toRadians(-90)))
                .build();

        intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
        intake.move(Intake.Angle.OUTTAKE);
        outtake.setAngle(Outtake.Angle.INTAKE);
        claw.setBothClaw(Claw.ClawState.CLOSED);

        while (opModeInInit())
        {
            intake.update();
            intake.updateClawState(intake.getClawStateLeft(), ClawSide.LEFT);
            intake.updateClawState(intake.getClawStateRight(), ClawSide.RIGHT);
            outtake.update();
            claw.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.DRIVE_TO_BOARD;
        drive.followTrajectoryAsync(goToBoard);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case DRIVE_TO_BOARD:
                    if (!drive.isBusy()) {
                        currentState = State.PLACE_DOUBLE_PIXEL;
                    }
                    break;
                case PLACE_DOUBLE_PIXEL:
                    runStateMachine = true;

                    if (donePlacinng) {
                        resetTime = getIntakeReleaseTime();
                        systemsState = SystemsState.RESET;

                        currentState = State.PARK;
                        drive.followTrajectoryAsync(park);
                    }
                    break;
                case PARK:
                    // close elevator and then intake

                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    // close systems
                    break;
            }

            if(runStateMachine)
            {
                systemsState = SystemsState.OPEN_SYSTEMS;
            }

            switch (systemsState)
            {
                case OPEN_SYSTEMS:
                    intake.move(Intake.Angle.INTAKE);

                    claw.setBothClaw(Claw.ClawState.CLOSED);
                    outtake.setAngle(Outtake.Angle.OUTTAKE);
                    elevator.setTarget(elevatorTarget);
                    intakeExtension.setTarget(intakeTarget);

                    releaseTime = getIntakeReleaseTime();

                    systemsState = SystemsState.RELEASE_PIXELS;
                    break;
                case RELEASE_PIXELS:
                    if((getIntakeReleaseTime() - releaseTime) >= releasePixelsDelay)
                    {
                        claw.setBothClaw(Claw.ClawState.OPEN);
                        intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH);

                        donePlacinng = true;
                    }
                    break;
                case RESET:
                    if((getIntakeReleaseTime() - resetTime) >= resetDelay)
                    {
                        intakeReleaseTime = getIntakeReleaseTime();
                        outtake.setAngle(Outtake.Angle.INTAKE);
                        elevator.setTarget(0);
                        intakeExtension.setTarget(0);

                        if(((getIntakeReleaseTime() - intakeReleaseTime) >= intakeResetDelay))
                        {
                            intake.move(Intake.Angle.OUTTAKE);
                        }
                        systemsState = SystemsState.IDLE;
                    }
                    break;
                default:
                    break;
            }

            drive.update();
            intakeExtension.update();
            intake.update();
            intake.updateClawState(intake.getClawStateLeft(), ClawSide.LEFT);
            intake.updateClawState(intake.getClawStateRight(), ClawSide.RIGHT);
            intakeExtension.update();
            outtake.update();
            elevator.update();
            claw.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    double getIntakeReleaseTime()
    {
        return timer.nanoseconds() / 1000000;
    }

}