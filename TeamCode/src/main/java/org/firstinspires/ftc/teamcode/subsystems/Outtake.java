package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class Outtake implements Subsystem{

    private final RobotHardware robot;

    public static double intakeHandPivot = 0.165, intakeClawPivot = 0.95;
    public static double outtakeHandPivot = 0.7, outtakeClawPivot = 1;
    public static double outtakeSpinIntake = 0.32, outtakeSpinOuttake = 0.89, outtakeSpin45 = 0.1505;
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
        OUTTAKE
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


        switch(type) {
            case CLAW:
                switch (angle){
                    case INTAKE:
                        this.robot.outtakeClawPivotServo.setPosition(intakeClawPivot);
                        this.robot.outtakeSpinServo.setPosition(outtakeSpinIntake);
                        break;
                    case OUTTAKE:
                        this.robot.outtakeClawPivotServo.setPosition(outtakeClawPivot);
                        this.robot.outtakeSpinServo.setPosition(outtakeSpinOuttake);
                                                break;
                }
                break;
            case HAND:
                switch (angle){
                    case INTAKE:
                        this.robot.outtakeHandLeftServo.setPosition(intakeHandPivot);
                        this.robot.outtakeHandRightServo.setPosition(intakeHandPivot);
                        break;
                    case OUTTAKE:
                        this.robot.outtakeHandLeftServo.setPosition(outtakeHandPivot);
                        this.robot.outtakeHandRightServo.setPosition(outtakeHandPivot);
                        break;
                }
                break;
        }
    }

}