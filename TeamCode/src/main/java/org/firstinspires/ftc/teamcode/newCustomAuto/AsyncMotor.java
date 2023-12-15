package org.firstinspires.ftc.teamcode.newCustomAuto;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class AsyncMotor {

    public DcMotorEx motor;
    private double power;

    final Object lock = new Object();

    public AsyncMotor(DcMotorEx motor){
        this.motor = motor;
    }

    public void setPowerAsync(double power){
        synchronized (lock) {
            this.power = power;
        }
    }

    public void updatePowerAsync(){
        synchronized (lock) {
            motor.setPower(power);
        }
    }
}