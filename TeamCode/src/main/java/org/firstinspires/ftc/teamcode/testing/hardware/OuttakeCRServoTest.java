package org.firstinspires.ftc.teamcode.testing.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;


@TeleOp(name="CR Servo Analog Test", group="tests")
@Config
public class OuttakeCRServoTest extends LinearOpMode {

    public static double target = 0;
    public static boolean PID = false;
    public static double MAX_POWER = 1;
    public static double kP = 0.3, kI = 0, kD = 0.00051;
    private BetterGamepad cGamepad;
    private PIDFController.PIDCoefficients pidCoefficients;
    PIDFController controller, controller2;
    CRServo crServo, crServo2;
    AnalogInput analogInput, analogInput2;
    AbsoluteAnalogEncoder absoluteAnalogEncoder, absoluteAnalogEncoder2;
    double power1 = 0, power2 = 0;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        crServo  = hardwareMap.get(CRServo.class, "s3");
        crServo2  = hardwareMap.get(CRServo.class, "s4");
        analogInput = hardwareMap.get(AnalogInput.class, "aT1");
        analogInput2 = hardwareMap.get(AnalogInput.class, "aT2");
        absoluteAnalogEncoder = new AbsoluteAnalogEncoder(analogInput);
        absoluteAnalogEncoder2 = new AbsoluteAnalogEncoder(analogInput2);

        this.cGamepad = new BetterGamepad(gamepad1);
        this.pidCoefficients = new PIDFController.PIDCoefficients();
        pidCoefficients.kP = kP;
        pidCoefficients.kI = kI;
        pidCoefficients.kD = kD;

        controller = new PIDFController(pidCoefficients);
        controller2 = new PIDFController(pidCoefficients);

        waitForStart();

        while (opModeIsActive()) {
            cGamepad.update();

            controller.updateError(AngleUnit.normalizeRadians(Math.toRadians(target) - (absoluteAnalogEncoder.getCurrentPosition()/2)));
            controller2.updateError(AngleUnit.normalizeRadians(Math.toRadians(target) - (absoluteAnalogEncoder2.getCurrentPosition()/2)));

            power1 = controller.update();
            power2 = controller2.update();

            if(gamepad1.left_stick_y != 0)
            {
                crServo.setPower(Range.clip(-this.cGamepad.left_stick_y, -MAX_POWER, MAX_POWER));
                crServo2.setPower(Range.clip(-this.cGamepad.left_stick_y, -MAX_POWER, MAX_POWER));
            }
            else if(!PID)
            {
                crServo.setPower(0);
                crServo2.setPower(0);
            }
            else
            {

//                if(power1 > 0)
//                {
//                    power2 = Math.abs(power2);
//                }
//                else if(power2 > 0)
//                {
//                    power1 = Math.abs(power1);
//                }
//                else if(power1 < 0)
//                {
//                    power2 = -Math.abs(power2);
//                }
//                else if(power2 < 0)
//                {
//                    power1 = -Math.abs(power1);
//                }

                //crServo.setPower(power1);
                //crServo2.setPower(power2);
            }

            telemetry.addData("1 pos", Math.toDegrees(absoluteAnalogEncoder.getCurrentPosition()));
            telemetry.addData("1 power", power1);
            telemetry.addData("2 pos", Math.toDegrees(absoluteAnalogEncoder.getCurrentPosition()));
            telemetry.addData("2 power", power2);
            telemetry.update();

            telemetry.update();
        }
    }
}
