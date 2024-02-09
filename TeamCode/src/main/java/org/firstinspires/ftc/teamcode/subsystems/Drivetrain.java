package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

@Config
public class Drivetrain {

    boolean blueAlliance;

    private BetterGamepad _cGamepad1;

    private final RobotHardware robot;
    Vector2d input;
    public static double maxPower = 1;
    public static double slowerSpin = 0.5;
    double power = 0, twist = 0;
    boolean slow = false;

    //Constructor
    public Drivetrain(Gamepad gamepad1, boolean blueAlliance)
    {
        this.robot = RobotHardware.getInstance();

        // gamepad helper to see if pressed button once
        this._cGamepad1 = new BetterGamepad(gamepad1);

        power = maxPower;

        this.blueAlliance = blueAlliance;

        resetAngle();
    }

    public void update() {
        _cGamepad1.update();

        input = new Vector2d(
                Range.clip(_cGamepad1.left_stick_y, -power, power),
                Range.clip(-_cGamepad1.left_stick_x, -power, power)
        ).rotated(robot.getAngle());

        twist = Range.clip(_cGamepad1.right_stick_x, -power * slowerSpin, power * slowerSpin);

        if(_cGamepad1.XOnce())
        {
            resetAngle();
        }

        robot.dtFrontLeftMotor.setPower(input.getX() + twist + input.getY());
        robot.dtBackLeftMotor.setPower(input.getX()  + twist - input.getY());
        robot.dtFrontRightMotor.setPower(input.getX()  - twist - input.getY());
        robot.dtBackRightMotor.setPower(input.getX()  - twist + input.getY());
    }


    public void resetAngle()
    {
        robot.imu.resetYaw();

        // check if we are blue/red alliance and set zero angle - For centric drive
        if(!blueAlliance)
        {
            robot.setImuOffset(-Math.PI);
        }
        else if(blueAlliance)
        {
            robot.setImuOffset(Math.PI);
        }
    }

    public void fast()
    {
        power = maxPower;
        slowerSpin = 0.9;
    }

    public void slow()
    {
        power = maxPower / 2;
        slowerSpin = 0.85;
    }
}
