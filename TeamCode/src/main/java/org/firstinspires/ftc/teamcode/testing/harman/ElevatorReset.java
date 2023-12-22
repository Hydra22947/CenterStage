package org.firstinspires.ftc.teamcode.testing.harman;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;


@TeleOp(name="Elevator Reset Test", group="tests")
@Config
public class ElevatorReset extends LinearOpMode {

    private RobotHardware robot = RobotHardware.getInstance();
    public static double stallCurrent = 9.2;
    public static double power = -0.5;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);

        waitForStart();

        robot.elevatorMotorLeft.setCurrentAlert(stallCurrent, CurrentUnit.AMPS);
        robot.elevatorMotorRight.setCurrentAlert(stallCurrent, CurrentUnit.AMPS);

        while(opModeIsActive())
        {
            if(robot.elevatorMotorLeft.isOverCurrent())
            {
                robot.elevatorMotorLeft.setPower(0);
            }
            else
            {
                robot.elevatorMotorLeft.setPower(power);
            }

            if(robot.elevatorMotorRight.isOverCurrent())
            {
                robot.elevatorMotorRight.setPower(0);
            }
            else
            {
                robot.elevatorMotorRight.setPower(power);
            }

            telemetry.addData("Left current: ", robot.elevatorMotorLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right current: ", robot.elevatorMotorRight.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }

    }
}
