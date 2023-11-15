package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Hang Test", group="tests")
@Config
public class HangTest extends LinearOpMode {

    private DcMotor motor1 = null;
    public static double power = 0.5;
    int target = 0;
    @Override
    public void runOpMode() {
        motor1  = hardwareMap.get(DcMotor.class, "mE");
        //this.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //this.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.right_trigger > 0)
            {
                //motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //target = motor1.getCurrentPosition();
                motor1.setPower(power);
            }
            else if(gamepad1.left_trigger > 0)
            {
                //motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //target = motor1.getCurrentPosition();
                motor1.setPower(-power);
            }

            if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0)
            {
                //this.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //motor1.setTargetPosition(target);
                //motor1.setPower(power);
                //motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor1.setPower(0);
            }
        }
    }
}
