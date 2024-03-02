package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class IntakeExtension implements Subsystem
{

    DcMotorEx extensionMotor;
    private final RobotHardware robot = RobotHardware.getInstance();
    public static double MAX_LEVEL = 1640;
    double currentTarget = 0;
    boolean usePID = true;
    public static double maxPower = 1;
    public static double kP = 0.008, kI = 0, kD = 0.01;

    Gamepad gamepad;
    BetterGamepad cGamepad;
    PIDFController controller;
    PIDFController.PIDCoefficients pidCoefficients = new PIDFController.PIDCoefficients();

    boolean isAuto, firstPID = false;
    public IntakeExtension(Gamepad gamepad, boolean isAuto)
    {
        this.isAuto = isAuto;

        // EXTENSION
        this.extensionMotor = robot.hardwareMap.get(DcMotorEx.class, "mE");
        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(isAuto)
        {
            this.extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else
        {
            extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        this.gamepad = gamepad;
        this.cGamepad = new BetterGamepad(gamepad);

        pidCoefficients.kP = kP;
        pidCoefficients.kI = kI;
        pidCoefficients.kD = kD;

        controller = new PIDFController(pidCoefficients);
    }



    public IntakeExtension(boolean isAuto)
    {
        this.isAuto = isAuto;

        // EXTENSION
        this.extensionMotor = robot.hardwareMap.get(DcMotorEx.class, "mE");
        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidCoefficients.kP = kP;
        pidCoefficients.kI = kI;
        pidCoefficients.kD = kD;

        controller = new PIDFController(pidCoefficients);
    }

    public void setFirstPID(boolean firstPID) {
        this.firstPID = firstPID;
    }

    public void update() {

        if(!firstPID)
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
                        extensionMotor.setPower(Range.clip(-gamepad.left_stick_y, -maxPower/2, maxPower/2));
                    }
                    else
                    {
                        extensionMotor.setPower(Range.clip(-gamepad.left_stick_y, -maxPower, maxPower));
                    }
                }
                else if(gamepad.left_stick_y != 0 && gamepad.left_stick_button)
                {
                    extensionMotor.setPower(Range.clip(-gamepad.left_stick_y, -maxPower/2, maxPower/2));
                }
                else
                {
                    extensionMotor.setPower(0);
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
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extensionMotor.setTargetPosition((int)currentTarget);

        extensionMotor.setPower(1);

        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setPidControl()
    {
        controller.updateError(currentTarget - extensionMotor.getCurrentPosition());

        extensionMotor.setPower(controller.update());
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
        return extensionMotor.getCurrentPosition();
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