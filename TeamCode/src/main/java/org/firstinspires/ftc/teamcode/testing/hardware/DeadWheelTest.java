package org.firstinspires.ftc.teamcode.testing.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotHardware;


@TeleOp(name="Dead Wheel Test", group="tests")
@Config
public class DeadWheelTest extends LinearOpMode {

    public Encoder par0;
    public Encoder par1;
    public Encoder perp;


    @Override
    public void runOpMode() {
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "mFR"))); // front
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "mFL"))); // right
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "mBR"))); // left

        par0.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("front", perp.getPositionAndVelocity().position);
            telemetry.addData("pod left", par0.getPositionAndVelocity().position);
            telemetry.addData("pod right", par1.getPositionAndVelocity().position);
            telemetry.update();
        }
    }
}
