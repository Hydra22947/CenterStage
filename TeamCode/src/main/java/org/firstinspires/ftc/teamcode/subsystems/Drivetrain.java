package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

@Config
public class Drivetrain{

    boolean blueAlliance;

    private BetterGamepad _cGamepad1;

    private final RobotHardware robot;
    Vector2d input;
    public static double maxPower = 1;
    public static double slowerSpin = 0.5;
    public double power = 0, twist = 0;
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

        double y = Range.clip(-_cGamepad1.left_stick_y, -power, power); // Remember, Y stick value is reversed
        double x = Range.clip(_cGamepad1.left_stick_x, -power, power);
        twist = Range.clip(_cGamepad1.right_stick_x, -power * slowerSpin, power * slowerSpin);

        if (_cGamepad1.XOnce()) {
            robot.imu.resetYaw();
        }

        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        robot.dtFrontLeftMotor.setPower(rotY + rotX + twist);
        robot.dtBackLeftMotor.setPower(rotY - rotX + twist);
        robot.dtFrontRightMotor.setPower(rotY - rotX - twist);
        robot.dtBackRightMotor.setPower(rotY + rotX - twist);
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
        power = maxPower / 2.5;
        slowerSpin = 0.8;
    }
}
