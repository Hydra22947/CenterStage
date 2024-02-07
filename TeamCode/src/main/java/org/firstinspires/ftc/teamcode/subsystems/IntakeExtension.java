package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class IntakeExtension implements Subsystem
{

    private final RobotHardware robot;
    public static double BASE = 0;
    public static double MAX_LEVEL = 1000;
    double currentTarget = 0;
    boolean usePID = true;
    public static double maxPower = 1;
    public static double kP = 0.0075, kI = 0, kD = 0.01;

    Gamepad gamepad;
    BetterGamepad cGamepad;
    PIDFController controller;
    PIDFController.PIDCoefficients pidCoefficients = new PIDFController.PIDCoefficients();

    boolean isAuto;
    public IntakeExtension(Gamepad gamepad)
    {
        this.robot = RobotHardware.getInstance();

        this.gamepad = gamepad;
        this.cGamepad = new BetterGamepad(gamepad);

        pidCoefficients.kP = kP;
        pidCoefficients.kI = kI;
        pidCoefficients.kD = kD;

        controller = new PIDFController(pidCoefficients);

        isAuto = false;
    }

    public IntakeExtension()
    {
        this.robot = RobotHardware.getInstance();

        pidCoefficients.kP = kP;
        pidCoefficients.kI = kI;
        pidCoefficients.kD = kD;

        controller = new PIDFController(pidCoefficients);

        isAuto = true;
    }

    public void update() {

        if(!isAuto)
        {
            cGamepad.update();

            if (usePID)
            {
                setPidControl();
            }
            else
            {
                if(gamepad.left_stick_y != 0 && !gamepad.left_stick_button)
                {
                    if((-gamepad.left_stick_y) < 0)
                    {
                        robot.extensionMotor.setPower(Range.clip(-gamepad.left_stick_y, -maxPower/2, maxPower/2));
                    }
                    else
                    {
                        robot.extensionMotor.setPower(Range.clip(-gamepad.left_stick_y, -maxPower, maxPower));
                    }
                }
                else if(gamepad.left_stick_y != 0 && gamepad.left_stick_button)
                {
                    robot.extensionMotor.setPower(Range.clip(-gamepad.left_stick_y, -maxPower/2, maxPower/2));
                }
                else
                {
                    robot.extensionMotor.setPower(0);
                }

            }
        }
        else
        {
            firstPID();
        }
    }

    void firstPID()
    {
        robot.extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.extensionMotor.setTargetPosition((int)currentTarget);

        robot.elevatorMotorRight.setPower(1);
        robot.elevatorMotorLeft.setPower(1);

        robot.extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void setPidControl()
    {
        controller.updateError(currentTarget - robot.extensionMotor.getCurrentPosition());

        robot.extensionMotor.setPower(controller.update());
    }

    public void setTarget(double target)
    {
        if(target > MAX_LEVEL)
        {
            this.currentTarget = MAX_LEVEL;
        }
        else
        {
            this.currentTarget = target;
        }
    }

    public double getTarget()
    {
        return this.currentTarget;
    }

    public double getPos()
    {
        return robot.extensionMotor.getCurrentPosition();
    }


    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }

    public void setAuto(boolean isAuto)
    {
        this.isAuto = isAuto;
    }

    @Override
    public void play() {

    }

    @Override
    public void loop(boolean allowMotors) {
        isAuto = true;
        update();
    }

    @Override
    public void stop() {

    }

    public PIDFController getController() {
        return controller;
    }

}