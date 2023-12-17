package org.firstinspires.ftc.teamcode.testing.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name="Dead Wheel Test", group="tests")
@Config
@Disabled
public class DeadWheelTest extends LinearOpMode {

    public DcMotorEx left;
    public DcMotorEx right;
    public DcMotorEx front;


    @Override
    public void runOpMode() {
        front = hardwareMap.get(DcMotorEx.class, "mFR"); // front
        right = hardwareMap.get(DcMotorEx.class, "mFL"); // right
        left = hardwareMap.get(DcMotorEx.class, "mBR"); // left

        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        front.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        left.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("front", front.getCurrentPosition());
            telemetry.addData("pod left", left.getCurrentPosition());
            telemetry.addData("pod right", right.getCurrentPosition());
            telemetry.update();
        }
    }
}
