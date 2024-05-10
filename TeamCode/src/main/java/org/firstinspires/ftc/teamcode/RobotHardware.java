package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServo;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class RobotHardware {

    public DrivetrainSubsystem drivetrain;
    public LiftSubsystem elevator;
    public OuttakeSubsystem outtake;
    public IntakeSubsystem intake;
    public DroneSubsystem drone;
    public IntakeExtensionSubsystem intakeExtension;

    //drivetrain
    public HardwareMap hardwareMap;
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;

    // lift
    public DcMotorEx liftMotorRight;
    public DcMotorEx liftMotorLeft;

    // intake
    public DcMotorEx intakeExtensionMotor;
    public BetterServo intakeClawLeftServo;
    public BetterServo intakeClawRightServo;
    public BetterServo intakeAngleServo;
    public BetterServo intakeHandPivotRightServo;
    public BetterServo intakeHandPivotLeftServo;
    public RevColorSensorV3 colorRight;
    public RevColorSensorV3 colorLeft;

    // outake
    public BetterServo outtakeClawLeftServo;
    public BetterServo outtakeClawRightServo;
    public BetterServo outtakeClawPivotServo;
    public BetterServo outtakeHandRightServo;
    public BetterServo outtakeHandLeftServo;


    public BetterServo planeServo;

    public MecanumDrive drive;

    // TODO: ADD x3 Distance Sensors, webcam

    // Telemetry storage
    public Telemetry telemetry;
    private double voltage = 0.0;
    private ElapsedTime voltageTimer;


    // singleton go brrrr
    private static RobotHardware instance = null;
    public boolean enabled;

    public IMU imu;

    private ArrayList<BetterSubsystem> subsystems;

    private double imuAngle, imuOffset = 0;


    boolean has2Pixels = false, closeRight = false, closeLeft = false;

    /**
     * Creating the singleton the first time, instantiating.
     */
    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    /**
     * Created at the start of every OpModeBlue.
     *
     * @param hardwareMap The HardwareMap of the robot, storing all hardware devices
     * @param telemetry Saved for later in the event FTC Dashboard used
     */
    public void init(final HardwareMap hardwareMap, final Telemetry telemetry, Pose2d pose) {
        this.hardwareMap = hardwareMap;

        drive = new MecanumDrive(hardwareMap, pose);
        voltageTimer = new ElapsedTime();

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.subsystems = new ArrayList<>();

        this.imu = drive.imu.get();

        // DRIVETRAIN
        this.dtBackLeftMotor = hardwareMap.get(DcMotorEx.class, "mBL");
        this.dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.dtBackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.dtFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "mFL");
        this.dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.dtFrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.dtBackRightMotor = hardwareMap.get(DcMotorEx.class, "mBR");
        this.dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.dtFrontRightMotor = hardwareMap.get(DcMotorEx.class, "mFR");
        this.dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // LIFT
        this.liftMotorRight = hardwareMap.get(DcMotorEx.class, "mER");
        this.liftMotorLeft = hardwareMap.get(DcMotorEx.class, "mEL");
        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // INTAKE
        this.intakeAngleServo = new BetterServo(hardwareMap.get(Servo.class, "sIA"));
        intakeAngleServo.setDirection(Servo.Direction.REVERSE);
        this.intakeClawLeftServo = new BetterServo(hardwareMap.get(Servo.class, "sICL"));
        this.intakeClawRightServo = new BetterServo(hardwareMap.get(Servo.class, "sICR"));
        this.intakeClawRightServo.setDirection(Servo.Direction.REVERSE);
        // COLOR/DS SENSORS
        this.colorRight = hardwareMap.get(RevColorSensorV3.class, "cR");
        this.colorLeft = hardwareMap.get(RevColorSensorV3.class, "cL");
        this.intakeExtensionMotor = hardwareMap.get(DcMotorEx.class, "mE");
        intakeExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.intakeExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // HAND
        this.intakeHandPivotRightServo = new BetterServo(hardwareMap.get(Servo.class, "sIHPR"));
        this.intakeHandPivotLeftServo = new BetterServo(hardwareMap.get(Servo.class, "sIHPL"));
        intakeHandPivotLeftServo.setDirection(Servo.Direction.REVERSE);

        // OUTTAKE
        this.outtakeClawLeftServo = new BetterServo(hardwareMap.get(Servo.class, "sCL"));
        this.outtakeClawLeftServo.setDirection(Servo.Direction.REVERSE);
        this.outtakeClawRightServo = new BetterServo(hardwareMap.get(Servo.class, "sCR"));
        this.outtakeClawPivotServo = new BetterServo(hardwareMap.get(Servo.class, "sC"));
        this.outtakeClawPivotServo.setDirection(Servo.Direction.REVERSE);
        this.outtakeHandLeftServo = new BetterServo(hardwareMap.get(Servo.class, "sHL"));
        this.outtakeHandRightServo = new BetterServo(hardwareMap.get(Servo.class, "sHR"));

        this.planeServo = new BetterServo(hardwareMap.get(Servo.class, "sP"));

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void init(final HardwareMap hardwareMap, final Telemetry telemetry)
    {
        init(hardwareMap, telemetry, new Pose2d(0,0,0));
    }

    public void init(final HardwareMap hardwareMap, final Telemetry telemetry, boolean debug, Gamepad gamepad1, Gamepad gamepad2)
    {
        init(hardwareMap, telemetry, new Pose2d(0,0,0));

        drivetrain = new DrivetrainSubsystem(gamepad1);
        elevator = new LiftSubsystem(gamepad2, debug);
        outtake = new OuttakeSubsystem();
        intake = new IntakeSubsystem();
        drone = new DroneSubsystem();
        intakeExtension = new IntakeExtensionSubsystem(gamepad2, debug);
        intake.setAngle(IntakeSubsystem.Angle.OUTTAKE);
        intake.updateState(IntakeSubsystem.ClawState.INDETERMINATE, ClawSide.BOTH);
        outtake.setBothClaw(OuttakeSubsystem.ClawState.INTAKE);
        outtake.setAngle(OuttakeSubsystem.Angle.INTAKE);

        intake.periodic();
        outtake.periodic();
        intakeExtension.periodic();
    }

    public void loopVoltage(HardwareMap hardwareMap)
    {
        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }
    }

    public double getVoltage() {
        return voltage;
    }

    public void read() {
        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        for (BetterSubsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for (BetterSubsystem subsystem : subsystems) {
            subsystem.write();
        }
    }


    public void periodic() {
        for (BetterSubsystem subsystem : subsystems) {
            subsystem.periodic();
        }
    }

    public void reset() {
        for (BetterSubsystem subsystem : subsystems) {
            subsystem.reset();
        }
    }

    public void addSubsystem(BetterSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    public void setExternalHeading(double value) {
    }

    public double getAngle() {
        return Angle.norm(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + imuOffset);
    }

    public void setImuOffset(double offset)
    {
        this.imuOffset = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + offset;
    }

    public double getImuOffset()
    {
        return imuOffset;
    }

    public boolean has2Pixels() {
        // check for colors and closed
        return has2Pixels;
    }

    public void setHas2Pixels(boolean has2Pixels) {
        this.has2Pixels = has2Pixels;
    }

    public boolean isCloseRight() {
        // check for colors and closed
        return closeRight;
    }

    public void closeRight(boolean closeRight) {
        this.closeRight = closeRight;
    }

    public boolean isCloseLeft() {
        // check for colors and closed
        return closeLeft;
    }

    public void closeLeft(boolean closeLeft) {
        this.closeLeft = closeLeft;
    }
}
