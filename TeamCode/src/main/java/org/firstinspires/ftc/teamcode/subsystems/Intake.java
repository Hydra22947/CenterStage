package org.firstinspires.ftc.teamcode.subsystems;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.values.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;

@Config
public class Intake extends BetterSubsystem {

    private final RobotHardware robot;
    public static double speed = 1;
    double target, currentTarget = speed;

    public Intake()
    {
        this.robot = RobotHardware.getInstance();
    }

    @Override
    public void periodic() {
        target = currentTarget;

        robot.intakeServoRight.setPower(currentTarget);
        robot.intakeServoLeft.setPower(currentTarget);
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

    public void setTarget(double target)
    {
        currentTarget = target;
    }
}