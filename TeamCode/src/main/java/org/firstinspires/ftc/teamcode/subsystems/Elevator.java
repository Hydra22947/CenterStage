package org.firstinspires.ftc.teamcode.subsystems;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.values.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;

@Config
public class Elevator extends BetterSubsystem {

    private final RobotHardware robot;
    public static double BASE_LEVEL = 420;
    public static double MAX_LEVEL = 1260;
    public static double INCREMENT = 105;
    double currentTarget = 0;
    boolean usePID = true;
    public static double maxPower = 0.85;
    public static double kP = .01, kI = 0, kD = 0.00001;

    Gamepad gamepad;
    BetterGamepad cGamepad;
    PIDFController controller, controller2;
    PIDFController.PIDCoefficients pidCoefficients = new PIDFController.PIDCoefficients();

    boolean isAuto = false;
    public Elevator(Gamepad gamepad)
    {
        this.robot = RobotHardware.getInstance();



        this.gamepad = gamepad;
        this.cGamepad = new BetterGamepad(gamepad);

        pidCoefficients.kP = kP;
        pidCoefficients.kI = kI;
        pidCoefficients.kD = kD;

        controller = new PIDFController(pidCoefficients);
        controller2 = new PIDFController(pidCoefficients);

        isAuto = false;
    }

    public Elevator()
    {
        this.robot = RobotHardware.getInstance();

        pidCoefficients.kP = kP;
        pidCoefficients.kI = kI;
        pidCoefficients.kD = kD;

        controller = new PIDFController(pidCoefficients);
        controller2 = new PIDFController(pidCoefficients);

        isAuto = true;
    }

    @Override
    public void periodic() {

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
                    robot.elevatorMotorRight.setPower(Range.clip(-gamepad.left_stick_y, -maxPower, maxPower));
                    robot.elevatorMotorLeft.setPower(Range.clip(-gamepad.left_stick_y, -maxPower, maxPower));
                }
                else if(gamepad.left_stick_y != 0 && gamepad.left_stick_button)
                {
                    robot.elevatorMotorRight.setPower(Range.clip(-gamepad.left_stick_y, -maxPower/2, maxPower/2));
                    robot.elevatorMotorLeft.setPower(Range.clip(-gamepad.left_stick_y, -maxPower/2, maxPower/2));
                }
                else
                {
                    robot.elevatorMotorRight.setPower(0);
                    robot.elevatorMotorLeft.setPower(0);
                }
            }
        }
        else
        {
            setPidControl();
        }
    }

    void setPidControl()
    {
        controller.updateError(currentTarget - robot.elevatorMotorRight.getCurrentPosition());
        controller2.updateError(currentTarget - robot.elevatorMotorLeft.getCurrentPosition());

        robot.elevatorMotorRight.setPower(controller.update());
        robot.elevatorMotorLeft.setPower(controller2.update());

        robot.telemetry.addData("target", currentTarget);
        robot.telemetry.addData("power right", controller.update());
        robot.telemetry.addData("power left", controller2.update());
        robot.telemetry.update();
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
        setPidControl();
        this.currentTarget = target;
    }

    public void increment()
    {
        currentTarget += INCREMENT;
    }

    public void decrement()
    {
        currentTarget -= INCREMENT;
    }

    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }
}