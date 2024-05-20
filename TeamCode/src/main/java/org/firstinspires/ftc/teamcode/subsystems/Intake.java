package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.jetbrains.annotations.NotNull;

@Config
public class Intake implements Subsystem {

    private final RobotHardware robot;
    public static double intakeHandPivot = 0.075, intakeAmmoPivot = 0.275;
    public static double outtakeHandPivot = 0.55, outtakeAmmoPivot = .72; // פורק מהשאיבה הזוויות-מתוקן
    public static double midHandPivot = 0.55, midAmmoPivot = .75;
    public static double midTeleOpHandPivot = 0.5, midTeleopAmmoPivot = 0.65;
    public static double top5HandPivot = .31, top5AmmoPivot = 0.13;
    public static double top5HandPivotAuto = 0.315, top5AmmoPivotAuto = 0.13; //auto ערימה של אוטונומי רק ה5
    public static double top54HandPivot = 0.29, top54AmmoPivot = 0.125; // ערימה של 54
    public static double top54HandPivotAuto = 0.29, top54AmmoPivotAuto = 0.135;
    public static double top43HandPivot = 0.28, top43AmmoPivot = 0.1; // auto
    public static double top32HandPivot = 0.235, top32AmmoPivot = 0.13; // ערימה של 32
    public static double top32HandPivotAuto = 0.18, top32AmmoPivotAuto = 0.025;
    public static double top21HandPivot = 0.195, top21AmmoPivot = 0.125; // auto

    public static double autoFixIntakeHandPivot = 0.68, autoFixIntakeAmmoPivot = 0.66;

    public static double STACK = 0.3;
    public static double RIGHT_SENSOR_ERROR = 0;
    public static double clickOffset = 0.1;
    double OFFSET_AMMO = 0;

    public static double openRight = 0.1, openLeft = 0.1;
    public static double closeRight = .275, closeLeft = 0.3; // סרבואים של תפיסה של שאיבה שהם פתוחים
    public static double indeterminateRight = 0.15, indeterminateLeft = 0.15; // סרבואים של תפיסה של שאיבה שהם פתוחים

    public static double closeCauseWallRight;
    public static double closeCauseWallLeft;

    public static void setSeeFarFrom(double seeFarFrom) {
        Intake.seeFarFrom = seeFarFrom;
    }

    public static double seeFarFrom = 3.5;
    public static final double maxSeeFarFrom = 3.5;
    public static final double minSeeFarFrom = 2;

    @Override
    public void play() {

    }

    @Override
    public void loop(boolean allowMotors) {
        updateState(Type.AMMO);
        updateState(Type.HAND);
    }

    @Override
    public void stop() {

    }

    public enum Angle {
        INTAKE,
        OUTTAKE,
        MID, TELEOP_MID,
        TOP_54, TOP_43, TOP_32, TOP_21, TOP_5,
        TOP_5_AUTO,
        TOP_54_AUTO, TOP_32_AUTO,
        AUTO_FIX_INTAKE
    }

    public enum ClawState {
        OPEN,
        INDETERMINATE,
        CLOSE,
    }

    public enum Type {
        AMMO,
        HAND
    }

    Angle angle = Angle.OUTTAKE;
    ClawState clawStateLeft = ClawState.OPEN;
    ClawState clawStateRight = ClawState.OPEN;


    public Intake() {
        this.robot = RobotHardware.getInstance();
    }

    public void update() {
        updateState(Type.AMMO);
        updateState(Type.HAND);


        if (checkIfPixelInRight(robot.colorRight)) {
            robot.closeRight(true);
        } else {
            robot.closeRight(false);
        }

        if (checkIfPixelIn(robot.colorLeft)) {
            robot.closeLeft(true);
        } else {
            robot.closeLeft(false);
        }


        if (checkIfPixelInRight(robot.colorRight) && checkIfPixelIn(robot.colorLeft)) {
            robot.setHas2Pixels(true);
        } else {
            robot.setHas2Pixels(false);
        }
    }

    public void move(Angle angle) {
        setAngle(angle);

        updateState(Type.AMMO);
        updateState(Type.HAND);
    }

    public void moveStack() {
        this.robot.intakeAngleServo.setPosition((getPosition(angle, Type.AMMO)) - STACK);
        this.robot.intakeHandPivotServo.setPosition(getPosition(Angle.INTAKE, Type.HAND));
    }


    public void returnStack() {
        this.robot.intakeAngleServo.setPosition((getPosition(angle, Type.AMMO) - STACK));
        this.robot.intakeHandPivotServo.setPosition(getPosition(angle, Type.HAND));
    }


    public Angle getAngle() {
        return angle;
    }

    public void setAngle(Angle angle) {
        this.angle = angle;

        updateState(Type.AMMO);
        updateState(Type.HAND);
    }

    public void updateState(@NotNull Type type) {
        double position = getPosition(angle, type);

        switch (type) {
            case AMMO:
                this.robot.intakeAngleServo.setPosition(position);
                break;
            case HAND:
                this.robot.intakeHandPivotServo.setPosition(position);
                break;
        }
    }

    public double updateClawState(@NotNull ClawState state, @NotNull ClawSide side) {
        double position = getClawStatePosition(state, side);

        switch (side) {
            case LEFT:
                robot.intakeClawLeftServo.setPosition(position);
                this.clawStateLeft = state;
                break;
            case RIGHT:
                robot.intakeClawRightServo.setPosition(position);
                this.clawStateRight = state;
                break;
            case BOTH:
                position = getClawStatePosition(state, ClawSide.LEFT);
                robot.intakeClawLeftServo.setPosition(position);
                this.clawStateRight = state;
                position = getClawStatePosition(state, ClawSide.RIGHT);
                robot.intakeClawRightServo.setPosition(position);
                this.clawStateLeft = state;
                break;
        }

        return position;
    }


    private double getClawStatePosition(ClawState state, ClawSide side) {
        switch (side) {
            case LEFT:
                switch (state) {
                    case CLOSE:
                        return closeLeft;
                    case OPEN:
                        return openLeft;
                    case INDETERMINATE:
                        return indeterminateLeft;
                    default:
                        return 0.0;
                }
            case RIGHT:
                switch (state) {
                    case CLOSE:
                        return closeRight;
                    case OPEN:
                        return openRight;
                    case INDETERMINATE:
                        return indeterminateRight;
                    default:
                        return 0.0;
                }
            default:
                return 0.0;
        }
    }

    public ClawState getClawStateLeft() {
        return clawStateLeft;
    }

    public ClawState getClawStateRight() {
        return clawStateRight;
    }

    public boolean checkIfPixelIn(RevColorSensorV3 sensor) {
        return -seeFarFrom <= sensor.getDistance(DistanceUnit.CM) && sensor.getDistance(DistanceUnit.CM) <= seeFarFrom;
    }

    public boolean checkIfPixelInRight(RevColorSensorV3 sensor) {
        return -(seeFarFrom + RIGHT_SENSOR_ERROR) <= sensor.getDistance(DistanceUnit.CM) && sensor.getDistance(DistanceUnit.CM) <= (seeFarFrom + RIGHT_SENSOR_ERROR);
    }

    public boolean closedClaw() {
        return clawStateLeft == ClawState.CLOSE && clawStateRight == ClawState.CLOSE;
    }

    private double getPosition(Angle angle, Type type) {
        switch (type) {
            case AMMO:
                switch (angle) {
                    case INTAKE:
                        return intakeAmmoPivot + OFFSET_AMMO;
                    case OUTTAKE:
                        return outtakeAmmoPivot + OFFSET_AMMO;
                    case MID:
                        return midAmmoPivot + OFFSET_AMMO;
                    case TELEOP_MID:
                        return midTeleopAmmoPivot + OFFSET_AMMO;
                    case TOP_21:
                        return top21AmmoPivot + OFFSET_AMMO;
                    case TOP_54:
                        return top54AmmoPivot + OFFSET_AMMO;
                    case TOP_32:
                        return top32AmmoPivot + OFFSET_AMMO;
                    case TOP_5:
                        return top5AmmoPivot + OFFSET_AMMO;
                    case TOP_43:
                        return top43AmmoPivot + OFFSET_AMMO;
                    case TOP_54_AUTO:
                        return top54AmmoPivotAuto + OFFSET_AMMO;
                    case TOP_5_AUTO:
                        return top5AmmoPivotAuto + OFFSET_AMMO;
                    case TOP_32_AUTO:
                        return top32AmmoPivotAuto + OFFSET_AMMO;
                    case AUTO_FIX_INTAKE:
                        return autoFixIntakeAmmoPivot + OFFSET_AMMO;

                    default:
                        return 0.0;

                }

            case HAND:
                switch (angle) {
                    case INTAKE:
                        return intakeHandPivot;
                    case OUTTAKE:
                        return outtakeHandPivot;
                    case MID:
                        return midHandPivot;
                    case TELEOP_MID:
                        return midTeleOpHandPivot + OFFSET_AMMO;
                    case TOP_21:
                        return top21HandPivot;
                    case TOP_32:
                        return top32HandPivot;
                    case TOP_54:
                        return top54HandPivot;
                    case TOP_5:
                        return top5HandPivot;
                    case TOP_43:
                        return top43HandPivot;
                    case TOP_54_AUTO:
                        return top54HandPivotAuto;
                    case TOP_32_AUTO:
                        return top32HandPivotAuto;
                    case TOP_5_AUTO:
                        return top5HandPivotAuto;
                    case AUTO_FIX_INTAKE:
                        return autoFixIntakeHandPivot;
                    default:
                        return 0.0;
                }
            default:
                return 0.0;
        }
    }


    public void setOFFSET_AMMO(double OFFSET_AMMO) {
        this.OFFSET_AMMO = OFFSET_AMMO;
    }
}