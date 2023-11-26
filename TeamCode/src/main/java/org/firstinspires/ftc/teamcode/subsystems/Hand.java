package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;

public class Hand extends BetterSubsystem
{
    private final RobotHardware robot;
    Servo servoArmRight, servoArmLeft, servoClaw;

    // Define claw dimensions
    double handLength = 1; // Length from the arm intersection point to the claw in CM
    double clawAngle = 0; // default angle of the claw servo
    public static double targetX = 50, targetZ = 100, targetClawAngle = 45;
    public static double kpArm = 1, kiArm = 0, kdArm = 0;
    public static double kpClaw = 1, kiClaw = 0, kdClaw = 0;

    // Variables for PID control
    double previousErrorArm = 0.0, integralArm = 0.0, previousErrorClaw = 0.0, integralClaw = 0.0;

    //Constructor
    public Hand()
    {
        this.robot = RobotHardware.getInstance();
    }

    @Override
    public void periodic() {
        // Calculate the angle for the arm
        double thetaArm = Math.atan2(targetZ, targetX);

        // Calculate the angle for the claw servo using 2D trigonometry
        double d = Math.sqrt(targetX * targetX + targetZ * targetZ);
        double thetaClaw = Math.atan2(targetClawAngle, d - handLength);

        // Perform PID control for the arm
        double errorArm = thetaArm - servoPosToRadians(servoArmRight.getPosition());
        integralArm += errorArm;
        double derivativeArm = errorArm - previousErrorArm;
        double outputArm = kpArm * errorArm + kiArm * integralArm + kdArm * derivativeArm;

        // Perform PID control for the claw
        double errorClaw = thetaClaw - clawAngle;
        integralClaw += errorClaw;
        double derivativeClaw = errorClaw - previousErrorClaw;
        double outputClaw = kpClaw * errorClaw + kiClaw * integralClaw + kdClaw * derivativeClaw;

        // Convert PID outputs to servo positions (adjust for your servo's range)
        double armServoPosition = Math.toDegrees(servoPosToRadians(servoArmRight.getPosition()) + outputArm)/270;
        //double armServoPosition = outputArm;
        double clawServoPosition = Math.toDegrees(clawAngle + outputClaw)/270;

        // Set servo positions
        moveArm(armServoPosition);
        servoClaw.setPosition(clawServoPosition);

        // Update previous error values
        previousErrorArm = errorArm;
        previousErrorClaw = errorClaw;

        // Update the current claw angle
        clawAngle = clawServoPosition;
    }

    public void setPos(double x, double z, double angle)
    {
        targetX = x;
        targetZ = z;
        targetClawAngle = angle;
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

    void moveArm(double pos)
    {
        servoArmLeft.setPosition(pos);
        servoArmRight.setPosition(pos);
    }

    double servoPosToRadians(double pos)
    {
        return Math.toRadians(pos);
    }
}
