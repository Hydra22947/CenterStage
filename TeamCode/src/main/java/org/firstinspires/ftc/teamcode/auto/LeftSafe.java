package org.firstinspires.ftc.teamcode.auto;
/*
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.auto.machines.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Stopwatch;
import org.openftc.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

@Config
@Autonomous(name = "LeftSafe", group = "Left")
public class LeftSafe extends FataOpMode {
    public static final Pose2d START_POSE = new
            Pose2d(-31.5, -63.0, Math.toRadians(-90.0));

    public static final Pose2d DEPOSIT_POSE = new
            Pose2d(-31.75, -14.0, Math.toRadians(175.0));

    public static final Pose2d INTAKE_POSE = new
            Pose2d(-38.0, -12.75, Math.toRadians(-179.0));

    public static double TURRET_ANGLE = 0.2875, SLIDES_THRESHOLD = 500,
            SLIDES_POWER = 0.5, SLIDES_CLOSE_POWER = -0.6;

    private TrajectorySequence lastTrajectory = null;

    private boolean intakingCone = false;
    private boolean transferringCone = false;
    private boolean goToDeposit = false;

    private int remainingCones = 5;
    private int detection = 1;

    private final Stopwatch intakeTimeout = new Stopwatch();
    private boolean missedCycle = false;
    private boolean skipDeposit = false;
    private boolean noDeposit = false;

    @Override
    protected void onInit() {
        deposit.setTurret(0.35);
        deposit.lockTurret(true);
        drive.resetIMU();
        drive.setPoseEstimate(START_POSE);

        intake.setBase(IntakeSubsystem.BasePosition.INIT);

        build(autoFSM());
        build(intakeFSM());
    }

    private State<AutoState> autoFSM() {
        return new StateMachineBuilder<AutoState>()

                .state(AutoState.DRIVE_TO_PRELOAD)
                .sample(drive::isNotBusy)
                .onEnter(() -> {
                    drive.followTrajectorySequenceAsync(
                            lastTrajectory = drive.trajectorySequenceBuilder(START_POSE)
                                    .setTangent(Math.toRadians(0.0))
                                    .splineToConstantHeading(new Vector2d(-10.75, -50.0), Math.toRadians(90.0),
                                            new MecanumVelocityConstraint(25.0, DriveConstants.TRACK_WIDTH,
                                                    DriveConstants.WHEEL_BASE, SampleMecanumDrive.LATERAL_MULTIPLIER),
                                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))

                                    .splineToSplineHeading(new Pose2d(-10.75, -31.0, Math.toRadians(-90.0)), Math.toRadians(90.0),
                                            new MecanumVelocityConstraint(25.0, DriveConstants.TRACK_WIDTH,
                                                    DriveConstants.WHEEL_BASE, SampleMecanumDrive.LATERAL_MULTIPLIER),
                                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))

                                    .addSpatialMarker(new Vector2d(-10.5, -35.33), () -> {
//                                        deposit.releaseCone();
//                                        withDelay(125, () -> lift.setVGuide(LiftSubsystem.VGuidePosition.CLOSED));
                                    })

                                    .build()
                    );

//                    withDelay(600, () -> {
//                        intake.setBase(IntakeSubsystem.BasePosition.CLOSED);
//                        intake.setClaw(IntakeSubsystem.ClawPosition.OPEN);
//                    });

                    withDelay(450, () -> {
//                        lift.setTarget(AutoLiftPositions.HIGH);
//                        withDelay(700, () -> {
//                            deposit.lockTurret(false);
//                            deposit.setTurret(0.285);
//                        });
//                        deposit.setBasePosition(DepositBasePosition.AUTO);
//                        withDelay(450, () -> lift.setVGuide(LiftSubsystem.VGuidePosition.CLOSED_LEFT));
                    });
                })

                .state(AutoState.DEPOSIT_PRELOAD)
//                .time(250)
                .instant()
                //.onEnter(deposit::releaseCone)
                .onExit(() -> {
//                    deposit.holdCone();
//                    deposit.setTurret(0.511);
//                    deposit.setBasePosition(DepositBasePosition.INTAKE);
//                    withDelay(75, () -> lift.setTarget(0));
//                    lift.setVGuide(LiftSubsystem.VGuidePosition.CLOSED);
                })

                .state(AutoState.TRANSITION_TO_CYCLES)
                .sample(drive::isNotBusy)
                .onEnter(() -> drive.followTrajectorySequenceAsync(
                        lastTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                                .setTangent(Math.toRadians(105.0))
                                .splineToSplineHeading(new Pose2d(-21.0, INTAKE_POSE.getY(), Math.toRadians(180.0)), Math.toRadians(180.0),
                                        new MecanumVelocityConstraint(45.0, DriveConstants.TRACK_WIDTH,
                                                DriveConstants.WHEEL_BASE, SampleMecanumDrive.LATERAL_MULTIPLIER),
                                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(() -> {
//                                    intake.setBase(IntakeHeight.values()[remainingCones - 1].basePosition);
//                                    withDelay(150, () -> {
//                                        deposit.holdCone();
//                                        deposit.setTurret(0.511);
//                                        lift.setVGuide(LiftSubsystem.VGuidePosition.CLOSED_LEFT);
//                                        deposit.setBasePosition(DepositBasePosition.INTAKE);
//                                        withDelay(75, () -> lift.setTarget(0));
//                                    });
                                })

                                .splineToSplineHeading(INTAKE_POSE.plus(new Pose2d(8.0, 0.0)), Math.toRadians(180.0),
                                        new MecanumVelocityConstraint(25.0, DriveConstants.TRACK_WIDTH,
                                                DriveConstants.WHEEL_BASE, SampleMecanumDrive.LATERAL_MULTIPLIER),
                                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))

                                .splineToSplineHeading(INTAKE_POSE.minus(new Pose2d(1.5, 0.0)), Math.toRadians(180.0),
                                        new MecanumVelocityConstraint(25.0, DriveConstants.TRACK_WIDTH,
                                                DriveConstants.WHEEL_BASE, SampleMecanumDrive.LATERAL_MULTIPLIER),
                                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))

                                .build()
                )).onExit(this::openIntake)
                .exitTo(() -> AutoState.WAIT_FOR_INTAKE)

                .state(AutoState.DRIVE_TO_INTAKE)
                .sample(drive::isNotBusy)
                .onEnter(() -> {
//                    intake.setBase(IntakeHeight.values()[remainingCones - 1].basePosition);
//                    openIntake();
                    withDelay(420, this::openIntake);

                    withDelay(0, () -> drive.followTrajectorySequenceAsync(
                            lastTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                                    .lineToLinearHeading(INTAKE_POSE.plus(new Pose2d(missedCycle ? -0.75 : 0.0, 0.05 * (5 - remainingCones))), new MecanumVelocityConstraint(
                                                    45.0, DriveConstants.TRACK_WIDTH, DriveConstants.WHEEL_BASE,
                                                    SampleMecanumDrive.LATERAL_MULTIPLIER),
                                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))

//                                    .addDisplacementMarker(16.5, this::openIntake)

                                    .build()
                    ));
                })//.onExit(this::openIntake)

                .state(AutoState.WAIT_FOR_INTAKE)
                .sample(() -> goToDeposit)
                .onEnter(() -> {
                    if (skipDeposit) openIntake();

                    skipDeposit = false;
                })

                .state(AutoState.WAIT_FOR_TRANSFER)
                .sample(() -> goToDeposit)
                .onExit(() -> {
                    if (noDeposit) {
//                        intake.setSlides(400);
//                        withDelay(300, () -> lift.setVGuide(LiftSubsystem.VGuidePosition.CLOSED));
//                        withDelay(400, () -> {
//                            lift.setTarget(0);
////                            intake.setSlides(0);
//                            intake.slidesOverridePower = SLIDES_CLOSE_POWER;
//                            intake.setBase(IntakeSubsystem.BasePosition.INIT);
//                            intake.setClaw(IntakeSubsystem.ClawPosition.OPEN);
//                            deposit.setBasePosition(DepositBasePosition.READY);
//                        });
                    }

                    goToDeposit = false;
                }).exitTo(() -> noDeposit ? AutoState.PARK : (skipDeposit ? AutoState.WAIT_FOR_INTAKE : null))

                .state(AutoState.DRIVE_TO_DEPOSIT)
                .time(350)
                .onEnter(() -> {
                   // lift.setTarget(AutoLiftPositions.MIDDLE /*+ 60*///)//;
  /*                  deposit.setTurret(TURRET_ANGLE);
                    deposit.setBasePosition(DepositBasePosition.AUTO);
                    if (remainingCones != 0) intake.setBase(IntakeHeight.values()[remainingCones - 1].basePosition);
                })

                .state(AutoState.PRE_DEPOSIT)
                .sample(drive::isNotBusy)
                .onEnter(() -> drive.followTrajectorySequenceAsync(
                        lastTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                                .lineToSplineHeading(DEPOSIT_POSE.plus(new Pose2d(0.0, 0.0085 * (5 - remainingCones))))
//                                .lineToConstantHeading(DEPOSIT_POSE.plus(new Pose2d(0.0, 0.0085 * (5 - remainingCones))).vec())

//                                .addDisplacementMarker(5.0, deposit::releaseCone)

                                .build()))

                .state(AutoState.DEPOSIT)
//                .time(125)
                .instant()
                .onEnter(() -> {
                    deposit.releaseCone();

                    if (runtime.seconds() >= 25.5) noDeposit = true;
                }).onExit(() -> withDelay(350, () -> {
                    deposit.holdCone();
                    deposit.setTurret(0.511);
                    if (remainingCones <= 0) {
                        intake.setSlides(400);
                        withDelay(200, () -> lift.setVGuide(LiftSubsystem.VGuidePosition.CLOSED));
                        withDelay(400, () -> {
                            lift.setTarget(0);
//                            intake.setSlides(0);
                            intake.slidesOverridePower = SLIDES_CLOSE_POWER;
                            intake.setBase(IntakeSubsystem.BasePosition.INIT);
                            intake.setClaw(IntakeSubsystem.ClawPosition.OPEN);
                            deposit.setBasePosition(DepositBasePosition.READY);
                        });
                    } else {
                        lift.setTarget(0);
                        deposit.setBasePosition(DepositBasePosition.INTAKE);
                    }
                })).exitTo(() -> {
//                    remainingCones == 0 ? null : AutoState.DRIVE_TO_INTAKE
                    if (runtime.seconds() >= 26.5 || remainingCones <= 0)
                        return null;

                    return AutoState.DRIVE_TO_INTAKE;
                })

                .state(AutoState.PARK)
                .sample(drive::isNotBusy)
                .onEnter(() -> {
                    TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(drive.getPoseEstimate());

                    if (detection == 3) builder.lineToConstantHeading(new Vector2d(-28.0, -12.0));

//                    intake.setBase(IntakeSubsystem.BasePosition.INIT);

//                    if (noDeposit) deposit.setBasePosition(DepositBasePosition.READY);

                    withDelay(500, () -> drive.followTrajectorySequenceAsync(
                            lastTrajectory = builder.lineToSplineHeading(new Pose2d(
                                            detection == 3 ? -11.0 :
                                                    (detection == 2 ?
                                                            -37.0 : -63.0), -12.75,
                                            Math.toRadians(detection == 3 ? 90.0 : detection == 2 ? -90.0 : 180.0)))

                                    .build()
                    ));

//                    intake.setSlides(250);
//                    withDelay(200, () -> lift.setVGuide(LiftSubsystem.VGuidePosition.CLOSED));
//                    withDelay(300, () -> intake.setSlides(0));
                })

                ;
    }

    private StateMachineBuilder<IntakeState> intakeFSM() {
        return new StateMachineBuilder<IntakeState>()

                .state(IntakeState.WAIT)
                .sample(() -> intakingCone)
                .onExit(() -> intakingCone = false)

                .state(IntakeState.OPEN_INTAKE)
//                .time(550)
                .sample(() -> intake.getSlidesPosition() > (remainingCones == 5 ?
                        SLIDES_THRESHOLD + 50 : SLIDES_THRESHOLD) || intakeTimeout.hasTimePassed(800))
                .onEnter(() -> {
                    if (remainingCones <= 0) return;

                    intakeTimeout.reset();

                    IntakeHeight intakeHeight = IntakeHeight
                            .values()[remainingCones - 1];

//                    //TODO
//                    remainingCones--;

                    intake.setBase(intakeHeight.basePosition);
//                    intake.setSlides(IntakeSubsystem.SlidesPosition.OPEN + 30);
//                    intake.setSlides(remainingCones == 5 ? 725 : 625);
                    intake.slidesOverridePower = SLIDES_POWER;
                    intake.setClaw(IntakeSubsystem.ClawPosition.OPEN);
                }).onExit(() -> intake.slidesOverridePower = 0)

                .state(IntakeState.WAIT_FOR_TRANSFER_2)
                .sample(() -> transferringCone)
                .onExit(() -> {
                    transferringCone = false;
                    withDelay(100, () -> {
                    });
                })

                .state(IntakeState.CLOSE_CLAW)
                .time(300)
                .onEnter(() -> intake.setClaw(
                        IntakeSubsystem.ClawPosition.CLOSED))
                .exitTo(() -> remainingCones > 0 ? null
                        : IntakeState.TRANSFER_2)

                .state(IntakeState.TRANSFER_1)
                .time(100)
                .onEnter(() -> intake.setBase(
                        IntakeSubsystem.BasePosition.CLOSED))

                .state(IntakeState.TRANSFER_2)
                .sample(() -> deposit.isHoldingCone() || intakeTimeout.hasTimePassed(800))
                .onEnter(() -> {
                    intakeTimeout.reset();
                    intake.setBase(IntakeSubsystem.BasePosition.CLOSED);
                    intake.setSlides(IntakeSubsystem.SlidesPosition.CLOSED);
                    deposit.setArrowPosition(ArrowPosition.INTAKE);
                    deposit.holdCone();
                }).onExit(() -> {
                    if (!deposit.isHoldingCone()) {
                        if (missedCycle) goToPark();
                        else missedCycle = true;

                        skipDeposit = true;
                    } else remainingCones--; // TODO
                }).exitTo(() -> {
                    if (!deposit.isHoldingCone()) {
                        goToDeposit = true;

                        return IntakeState.WAIT;
                    }

                    return null;
                })

                .state(IntakeState.TRANSFER_3)
                .time(50)
                .onEnter(() -> intake.setBase(IntakeSubsystem.BasePosition.SUPER_CLOSED))

                .state(IntakeState.OPEN_CLAW)
                .time(100)
                .onEnter(() -> {
                    intake.setClaw(IntakeSubsystem.ClawPosition.OPEN);
                    deposit.setArrowPosition(ArrowPosition.HOLD);
                }).onExit(() -> goToDeposit = true)

                .exitTo(() -> IntakeState.WAIT)

                ;
    }

    private void openIntake() {
        intakingCone = true;
        transferringCone = true;
    }

    private void goToPark() {
        drive.followTrajectorySequence(
                lastTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                        .lineToSplineHeading(new Pose2d(
                                detection == 1 ? -13.0 :
                                        (detection == 2 ?
                                                -37.0 : -63.0), -12.0,
                                Math.toRadians(detection != 3 ? -90.0 : 180.0)))

                        .addDisplacementMarker(() -> {
                            intake.setSlides(400);
                            withDelay(300, () -> lift.setVGuide(LiftSubsystem.VGuidePosition.CLOSED));
                            withDelay(400, () -> intake.setSlides(0));
                            withDelay(700, this::stopOpMode);
                        })
*/
//                        .addDisplacementMarker(this::stopOpMode)
/*
                        .build()
        );
    }

    @Override
    protected void initLoop() {
        AprilTagDetection id = aprilTagDetector.getTagOfInterest();
        if (id != null) detection = id.id;
    }

    @Override
    protected void playLoop() {
        if (intake.slidesOverridePower == SLIDES_CLOSE_POWER &&
                intake.getSlidesPosition() <= 5) intake.slidesOverridePower = 0;
    }

    public enum IntakeHeight {
        ONE(IntakeSubsystem.BasePosition.OPEN,
                IntakeSubsystem.SlidesPosition.OPEN),

        TWO(IntakeSubsystem.BasePosition.TWO_CONES,
                IntakeSubsystem.SlidesPosition.TWO_CONES),

        THREE(IntakeSubsystem.BasePosition.THREE_CONES,
                IntakeSubsystem.SlidesPosition.THREE_CONES),

        FOUR(IntakeSubsystem.BasePosition.FOUR_CONES,
                IntakeSubsystem.SlidesPosition.FOUR_CONES),

        FIVE(IntakeSubsystem.BasePosition.FIVE_CONES,
                IntakeSubsystem.SlidesPosition.FIVE_CONES),
        ;

        public final IntakeSubsystem.BasePosition basePosition;
        public final int slidesPosition;

        IntakeHeight(IntakeSubsystem.BasePosition basePosition,
                     int slidesPosition) {
            this.basePosition = basePosition;
            this.slidesPosition = slidesPosition;
        }
    }

    public enum AutoState {
        DRIVE_TO_PRELOAD,
        DEPOSIT_PRELOAD,
        TRANSITION_TO_CYCLES,
        DRIVE_TO_INTAKE,
        WAIT_FOR_INTAKE,
        WAIT_FOR_TRANSFER,
        DRIVE_TO_DEPOSIT,
        PRE_DEPOSIT,
        DEPOSIT,
        PARK
    }

    public enum IntakeState {
        WAIT,
        BEFORE_OPEN_INTAKE,
        OPEN_INTAKE,
        CLOSE_CLAW,
        TRANSFER_1,
        WAIT_FOR_TRANSFER_2,
        PUSH_SLIDES,
        TRANSFER_2,
        TRANSFER_3,
        OPEN_CLAW
    }
}*/