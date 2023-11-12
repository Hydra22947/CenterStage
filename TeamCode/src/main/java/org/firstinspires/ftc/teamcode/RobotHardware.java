package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.values.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterEncoder;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServo;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;

import java.util.ArrayList;
import java.util.Arrays;

import javax.annotation.Nonnegative;

@Config
public class RobotHardware {

    //drivetrain
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;

    // other motors
    public DcMotorEx elevatorMotor;
    public DcMotorEx intakeMotor;

    // claw
    public BetterServo clawLeftServo;
    public BetterServo clawRightServo;
    public BetterServo clawPivotServo;
    public DigitalChannel breambeamRight;  // Touch sensor Object
    public DigitalChannel breambeamLeft;  // Touch sensor Object


    // hand
    public BetterServo handRightServo;
    public BetterServo handLeftServo;

    //Intake Extension
    public BetterServo extensionRightServo;
    public BetterServo extensionLeftServo;

    // TODO: ADD x3 Distance Sensors, webcam

    // odo pod encoders
    public BetterEncoder podLeft;
    public BetterEncoder podRight;
    public BetterEncoder podFront;

    // hardwareMap storage
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // singleton go brrrr
    private static RobotHardware instance = null;
    public boolean enabled;

    public IMU imu;

    private ArrayList<BetterSubsystem> subsystems;

    private double imuAngle, imuOffset = 0;

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
     * Created at the start of every OpMode.
     *
     * @param hardwareMap The HardwareMap of the robot, storing all hardware devices
     * @param telemetry Saved for later in the event FTC Dashboard used
     */
    public void init(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        if (Globals.USING_DASHBOARD) {
            this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        } else {
            this.telemetry = telemetry;
        }

        this.subsystems = new ArrayList<>();

        this.imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        this.imu.initialize(parameters);

        // DRIVETRAIN
//        this.dtBackLeftMotor = hardwareMap.get(DcMotorEx.class, "mBL");
//        this.dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.dtBackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        this.dtFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "mFL");
//        this.dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.dtFrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        this.dtBackRightMotor = hardwareMap.get(DcMotorEx.class, "mBR");
//        this.dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        this.dtFrontRightMotor = hardwareMap.get(DcMotorEx.class, "mFR");
//        this.dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // ELEVATOR
//        this.elevatorMotor = hardwareMap.get(DcMotorEx.class, "mE");
//
//        // INTAKE
//        this.intakeMotor = hardwareMap.get(DcMotorEx.class, "mI");


        breambeamRight = hardwareMap.get(DigitalChannel.class, "bbR");
        breambeamLeft = hardwareMap.get(DigitalChannel.class, "bbL");


        // CLAW
        this.clawLeftServo = new BetterServo(hardwareMap.get(Servo.class, "sCL"));
        this.clawLeftServo.setDirection(Servo.Direction.REVERSE);
        this.clawRightServo = new BetterServo(hardwareMap.get(Servo.class, "sCR"));
        this.clawPivotServo = new BetterServo(hardwareMap.get(Servo.class, "sC"));

        // HAND
        this.handLeftServo = new BetterServo(hardwareMap.get(Servo.class, "sHL"));
        this.handRightServo = new BetterServo(hardwareMap.get(Servo.class, "sHR"));

        //INTAKE EXTENSION
     //   this.extensionRightServo = new BetterServo(hardwareMap.get(Servo.class, "sER"));
       // this.extensionLeftServo = new BetterServo(hardwareMap.get(Servo.class, "sEL"));

//        this.podLeft = new BetterEncoder(new MotorEx(hardwareMap, "").encoder); // TODO: where the fuck did i connect them
//        this.podFront = new BetterEncoder(new MotorEx(hardwareMap, "").encoder);
//        this.podRight = new BetterEncoder(new MotorEx(hardwareMap, "").encoder);
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

    // TODO: add offset if needed
    // imuAngle - imuOffset;
    @Nonnegative
    public double getAngle() {
        return imuAngle - imuOffset;
    }

    public void setImuOffset(double offset)
    {
        this.imuOffset = offset;
    }
}
