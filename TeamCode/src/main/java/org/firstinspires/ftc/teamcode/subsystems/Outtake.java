package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.values.ClawSide;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class Outtake extends BetterSubsystem {

    private final RobotHardware robot;

    public static double intakeHandPivot = 0.08, intakeClawPivot = 0.04;
    public static double outtakeHandPivot = 0.5, outtakeClawPivot = 0.58;

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

    public Angle clawAngle = Angle.OUTTAKE;
    public Angle handAngle = Angle.OUTTAKE;
    Intake intake;
    Claw claw;

    public Outtake(Intake intake, Claw claw)
    {
        this.robot = RobotHardware.getInstance();
        this.intake = intake;
        this.claw = claw;
    }

    @Override
    public void periodic() {
        updateState(clawAngle, Type.CLAW);
        updateState(handAngle, Type.HAND);

        if(robot.isReadyToTransferPixels() && clawAngle == Angle.INTAKE && handAngle == Angle.INTAKE)
        {
            intake.intakeMove(Intake.powerTransfer);
            // until pixels see beam(already happens in opmode) because of peridoric
            if(!claw.checkAndClose(robot.breakbeamRight, ClawSide.RIGHT) && !claw.checkAndClose(robot.breakbeamLeft, ClawSide.LEFT))
            {
                clawAngle = Angle.OUTTAKE;
                handAngle = Angle.OUTTAKE;
            }

        }
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {

    }

    public void setClaw(Angle claw) {
        this.clawAngle = claw;
    }

    public void setHand(Angle hand) {
        this.handAngle = hand;
    }

    public void updateState(@NotNull Angle angle, @NotNull Type type) {
        double position = getPosition(angle, type);

        switch(type) {
            case CLAW:
                this.robot.outtakeClawPivotServo.setPosition(position);
                break;
            case HAND:
                this.robot.outtakeHandRightServo.setPosition(position);
                this.robot.outttakeHandLeftServo.setPosition(position);
                break;
        }
    }

    private double getPosition(Angle angle, Type type)
    {
        switch (type)
        {
            case CLAW:
                switch (angle) {
                    case INTAKE:
                        return intakeClawPivot;
                    case OUTTAKE:
                        return outtakeClawPivot;
                    default:
                        return 0.0;

                }

            case HAND:
                switch (angle) {
                    case INTAKE:
                        return intakeHandPivot;
                    case OUTTAKE:
                        return outtakeHandPivot;
                    default:
                        return 0.0;
                }
            default:
                return 0.0;
        }
    }

}