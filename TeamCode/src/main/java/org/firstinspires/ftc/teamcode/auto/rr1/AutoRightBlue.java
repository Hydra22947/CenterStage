package org.firstinspires.ftc.teamcode.auto.rr1;

// RR-specific imports

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.old_with_cycles.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.rr1.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.rr1.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.auto.rr1.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@Autonomous(name = "Auto Blue Left")
public class AutoRightBlue extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    ElapsedTime time;

    // subsystems
    MecanumDrive drivetrain;
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;
    AutoConstants autoConstants;


    DepositActions depositActions;
    PlacePurpleActions placePurpleActions;
    UpdateActions updateActions;

    @Override
    public void runOpMode() {
        time = new ElapsedTime();
        drivetrain = new MecanumDrive(hardwareMap, autoConstants.startPoseBlueLeft);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, true);

        autoConstants = new AutoConstants();

        elevator = new Elevator(true);
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension(true);

        intakeExtension.setAuto(true);
        elevator.setAuto(true);

        depositActions = new DepositActions(elevator, intake, claw, outtake);
        placePurpleActions = new PlacePurpleActions(intake, intakeExtension);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);

        SequentialAction deposit = new SequentialAction(
                depositActions.readyForDeposit(),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD),
                depositActions.retractDeposit()
        );
        SequentialAction readyForDeposit = new SequentialAction(
                depositActions.readyForDeposit()
        );
        SequentialAction placePurplePixel = new SequentialAction(
                placePurpleActions.placePurpleMid()
        );


        SequentialAction retractDeposit = new SequentialAction(
                depositActions.retractDeposit()
        );

        SequentialAction retractIntake = new SequentialAction(
                placePurpleActions.retract()
        );


        while (opModeInInit() && !isStopRequested()) {
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            intake.setAngle(Intake.Angle.OUTTAKE);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addLine("Initialized");
        }

        waitForStart();

        if (isStopRequested()) return;

        Action traj =
                drivetrain.actionBuilder(drivetrain.pose)
                        .lineToY(12)
                        .stopAndAdd(placePurplePixel)
                        .waitSeconds(1)
                        .stopAndAdd(retractIntake)
//                        .stopAndAdd(readyForDeposit)
                        .setTangent(0)
                        //Place Preload on board
                        .splineToLinearHeading(new Pose2d(51, 40, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(.1)
                        .stopAndAdd(deposit)
                        .waitSeconds(.1)
                        .stopAndAdd(retractDeposit)
                        //Park
                        .setTangent(Math.toRadians(90))
                        .lineToY(60)
                        .build();

        waitForStart();

        if (isStopRequested()) return;

        runBlocking(new ParallelAction(
                traj,
                updateActions.updateSystems()
        ));

    }


}