package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class Outtake implements Subsystem {

    private final RobotHardware robot;

    public static double almostIntakeHandPivot = 0.35, intakeHandPivot = 0.2, intakeClawPivot = 0;
    public static double outtakeHandPivot = 0.7, outtakeClawPivot = .4;
    public static double floorHandPivot = 0.85, floorClawPivot = 0.5, goBackRelease = 0.05;

    public static double outtakeSpinIntake = 0.885, outtakeSpinOuttake = 0.885, outtakeSpin45 = 0.1505;
    double defaultOuttakeSpinOuttake = 0.875, defaultOuttakeHandPivot = 0.7;
    public static double outtakeSpinDouble = 0.0325;
    public static double hangHand = 0.6, handClaw = .4;

    public static double power = 1;

    public static void setOuttakeHandPivot(double outtakeHandPivot) {
        Outtake.outtakeHandPivot = outtakeHandPivot;
    }

    @Override
    public void play() {

    }

    @Override
    public void loop(boolean allowMotors) {
        update();
    }

    @Override
    public void stop() {

    }

    public enum Angle {
        INTAKE,
        ALMOST_INTAKE,
        OUTTAKE,
        FLOOR,
        HANG,

    }

    public enum Type {
        CLAW,
        HAND
    }

    Angle angle = Angle.INTAKE;

    public Outtake() {
        this.robot = RobotHardware.getInstance();
    }


    public void update() {
        updateState(Type.HAND);
        updateState(Type.CLAW);

    }

    public void setAngle(@NotNull Angle angle) {
        this.angle = angle;

        updateState(Type.HAND);
        updateState(Type.CLAW);
    }

    public void updateState(@NotNull Type type) {


        switch (type) {
            case CLAW:
                switch (angle) {
                    case INTAKE:
                    case ALMOST_INTAKE:
                        this.robot.outtakeClawPivotServo.setPosition(intakeClawPivot);
                        this.robot.outtakeSpinServo.setPosition(outtakeSpinIntake);
                        break;
                    case OUTTAKE:
                        this.robot.outtakeClawPivotServo.setPosition(outtakeClawPivot);
                        this.robot.outtakeSpinServo.setPosition(outtakeSpinOuttake);
                        break;
                    case FLOOR:
                        this.robot.outtakeClawPivotServo.setPosition(floorClawPivot);
                        break;
                    case HANG:
                        this.robot.outtakeClawPivotServo.setPosition(handClaw);
                        break;
                }
                break;
            case HAND:
                switch (angle) {
                    case ALMOST_INTAKE:
                        this.robot.outtakeHandLeftServo.setPosition(almostIntakeHandPivot);
                        this.robot.outtakeHandRightServo.setPosition(almostIntakeHandPivot);
                        break;
                    case INTAKE:
                        this.robot.outtakeHandLeftServo.setPosition(intakeHandPivot);
                        this.robot.outtakeHandRightServo.setPosition(intakeHandPivot);
                        break;
                    case OUTTAKE:
                        this.robot.outtakeHandLeftServo.setPosition(outtakeHandPivot);
                        this.robot.outtakeHandRightServo.setPosition(outtakeHandPivot);
                        break;
                    case FLOOR:
                        this.robot.outtakeHandRightServo.setPosition(floorHandPivot);
                        this.robot.outtakeHandLeftServo.setPosition(floorHandPivot);
                        break;
                    case HANG:
                        this.robot.outtakeHandRightServo.setPosition(hangHand);
                        this.robot.outtakeHandLeftServo.setPosition(hangHand);
                        break;
                }
                break;
        }
    }

    public void spinOuttake(int direction)
    {
        outtakeSpinOuttake += (outtakeSpin45 * direction);

        outtakeSpinOuttake = Math.max(0, Math.min(1, outtakeSpinOuttake));

        this.robot.outtakeSpinServo.setPosition(outtakeSpinOuttake);
    }

    public void releasePixel()
    {
        outtakeHandPivot -= goBackRelease;
    }
    public void resetOuttake()
    {
        outtakeSpinOuttake = defaultOuttakeSpinOuttake;
        outtakeHandPivot = defaultOuttakeHandPivot;
    }

}