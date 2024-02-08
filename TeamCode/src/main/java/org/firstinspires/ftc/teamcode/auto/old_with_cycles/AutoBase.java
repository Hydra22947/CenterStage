package org.firstinspires.ftc.teamcode.auto.old_with_cycles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.machines.StateMachine;
import org.firstinspires.ftc.teamcode.auto.machines.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.ActionTimer;
import org.firstinspires.ftc.teamcode.util.Stopwatch;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

public class AutoBase extends LinearOpMode {
    protected SampleMecanumDrive drive;
    protected Claw claw;
    protected Elevator elevator;
    protected Intake intake;
    protected IntakeExtension intakeExtension;
    protected Outtake outtake;
//    protected AprilTagDetector aprilTagDetector;

    protected final List<Subsystem> subsystems = new ArrayList<>();
    protected final List<StateMachine<?>> stateMachineList = new ArrayList<>();
    protected final Queue<ActionTimer> actionTimerList = new ConcurrentLinkedQueue<>();
    private final VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

    private boolean shouldStop = false;

    protected final ElapsedTime runtime = new ElapsedTime();

    private static AutoBase instance;

    @Override
    public void runOpMode() {
        instance = this;

        boolean autonomous = getClass().isAnnotationPresent(Autonomous.class);

//        if (autonomous) {
//            LiftSubsystem.shouldReset = true;
//        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        subsystems.addAll(Arrays.asList(
                drive = new SampleMecanumDrive(hardwareMap),
                claw = new Claw(),
                elevator = new Elevator(true),
                intake = new Intake(),
                intakeExtension = new IntakeExtension(true),
                outtake = new Outtake()
        ));


//        if (autonomous) subsystems.add(aprilTagDetector =
//                new AprilTagDetector(hardwareMap, telemetry));

        onInit();

        while (!isStarted() && !isStopRequested()) {
//            if (autonomous && aprilTagDetector.getTagOfInterest() != null) telemetry.addLine(
//                    aprilTagDetector.getTagOfInterest().toString());

            subsystems.forEach(s -> s.loop(false));

            initLoop();

            telemetry.update();


            subsystems.forEach(Subsystem::play);
            runtime.reset();

            onPlay();

            Stopwatch cycleTimeStopwatch = new Stopwatch();

            while (opModeIsActive() && !isStopRequested() && !shouldStop) {
                cycleTimeStopwatch.reset();

                subsystems.forEach(s -> s.loop(true));
                stateMachineList.forEach(StateMachine::update);
                stateMachineList.removeIf(StateMachine::isDone);
                actionTimerList.removeIf(ActionTimer::update);

                playLoop();

                telemetry.update();
            }

            subsystems.forEach(Subsystem::stop);

            onStop();
        }
    }

    protected void onInit() {}

    protected void initLoop() {}

    protected void onPlay() {}

    protected void playLoop() {}

    protected void onStop() {}

    protected void stopOpMode() {
        shouldStop = true;
    }

    protected <E extends Enum<E>> void build(StateMachineBuilder<E> stateMachineBuilder) {
        stateMachineList.add(stateMachineBuilder.build());
    }

    protected void withDelay(long ms, Runnable whenTimer) {
        actionTimerList.add(new ActionTimer(ms, whenTimer));
    }

    public SampleMecanumDrive getDrive() {
        return drive;
    }

    public Claw getClaw() {
        return claw;
    }
    public Elevator getElevator() {
        return elevator;
    }
    public Intake getIntake() {
        return intake;
    }
    public IntakeExtension getIntakeExtension() {
        return intakeExtension;
    }
    public Outtake getOuttake() {
        return outtake;
    }

    public static AutoBase getInstance() {
        return instance;
    }
}