package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class Outtake implements Subsystem{

    private final RobotHardware robot;

    public static double intakeHandPivot = 0.75, intakeClawPivot = 0.095;
    public static double outtakeHandPivot = 0.29, outtakeClawPivot = 0.935;
    public static double outtakeHandPivotLong = .41, outtakeClawPivotLong = .89;
    public static double outtakeSpinDefault = 0.31, outtakeSpin45 = 0.1505;
    public static double outtakeSpinDouble = 0.0325;

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

    public enum Angle
    {
        INTAKE,
        OUTTAKE,
        OUTTAKE_LONG
    }

    public enum Type
    {
        CLAW,
        HAND
    }

    Angle angle = Angle.INTAKE;
    public Outtake()
    {
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

        this.robot.outtakeSpinServo.setPosition(outtakeSpinDefault);

        switch(type) {
            case CLAW:
                switch (angle){
                    case INTAKE:
                        this.robot.outtakeClawPivotServo.setPosition(intakeClawPivot);
                        break;
                    case OUTTAKE:
                        this.robot.outtakeClawPivotServo.setPosition(outtakeClawPivot);
                        break;
                    case OUTTAKE_LONG:
                        this.robot.outtakeClawPivotServo.setPosition(outtakeClawPivotLong);
                        break;
                }
                break;
            case HAND:
                switch (angle){
                    case INTAKE:
                        this.robot.outtakeHandServo.setPosition(intakeHandPivot);
                        break;
                    case OUTTAKE:
                        this.robot.outtakeHandServo.setPosition(outtakeHandPivot);
                        break;
                    case OUTTAKE_LONG:
                        this.robot.outtakeHandServo.setPosition(outtakeHandPivotLong);
                        break;
                }
                break;
        }
    }

}