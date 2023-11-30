package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.Globals;

@Config
public class Drivetrain {

    boolean redAlliance;

    private BetterGamepad _cGamepad1;

    private final RobotHardware robot;

    //Constructor
    public Drivetrain(Gamepad gamepad1, boolean redAlliance)
    {
        this.robot = RobotHardware.getInstance();

        // gamepad helper to see if pressed button once
        this._cGamepad1 = new BetterGamepad(gamepad1);

        this.redAlliance = redAlliance;

        resetAngle();
    }

    public void update() {
        _cGamepad1.update();
        double y = -_cGamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = _cGamepad1.left_stick_x;
        double rx = _cGamepad1.right_stick_x * 0.9;

        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        robot.dtFrontLeftMotor.setPower(frontLeftPower);
        robot.dtBackLeftMotor.setPower(backLeftPower);
        robot.dtFrontRightMotor.setPower(frontRightPower);
        robot.dtBackRightMotor.setPower(backRightPower);
    }


    public void resetAngle()
    {
        robot.imu.resetYaw();

//        // check if we are blue/red alliance and set zero angle - For centric drive
//        if(!redAlliance)
//        {
//            robot.setImuOffset((Math.PI - Math.PI / 2));
//        }
//        else if(redAlliance)
//        {
//            robot.setImuOffset(Math.PI + Math.PI );
//        }
        robot.telemetry.addData("RESET", 0);
        robot.telemetry.update();
    }

}
