package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class Outtake {

    private final RobotHardware robot;

    public static double intakeHandPivot = 0.2, intakeClawPivot = 0.131;
    public static double outtakeHandPivot = 0.65, outtakeClawPivot = .98;

    public static double power = 1;

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
        updateState(Type.CLAW);
        updateState(Type.HAND);
    }

    public void setAngle(@NotNull Angle angle) {
        this.angle = angle;
    }

    public void updateState(@NotNull Type type) {

        switch(type) {
            case CLAW:
                switch (angle){
                    case INTAKE:
                        this.robot.outtakeClawPivotServo.setPosition(intakeClawPivot);
                        break;
                    case OUTTAKE:
                        this.robot.outtakeClawPivotServo.setPosition(outtakeClawPivot);
                        break;
                }
                break;
            case HAND:
                switch (angle){
                    case INTAKE:
                        this.robot.outtakeHandRightServo.setPosition(intakeHandPivot);
                        this.robot.outtakeHandLeftServo.setPosition(intakeHandPivot);
                        break;
                    case OUTTAKE:
                        this.robot.outtakeHandRightServo.setPosition(outtakeHandPivot);
                        this.robot.outtakeHandLeftServo.setPosition(outtakeHandPivot);
                        break;
                }
                break;
        }
    }

}