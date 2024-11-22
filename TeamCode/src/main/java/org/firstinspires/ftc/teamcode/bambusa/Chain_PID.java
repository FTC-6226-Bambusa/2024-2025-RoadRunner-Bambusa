package org.firstinspires.ftc.teamcode.bambusa;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Chain_PID {

    private double ticks_in_degree = 300 / 90.0;

    private double integralSum = 0;
    private double lastError = 0;

    private double Kp = 0, Ki = 0, Kd = 0, f = 0;

    private ElapsedTime timer = new ElapsedTime();

    public Chain_PID(double p, double i, double d, double f) {
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
        this.f = f;
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();

        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * this.Kp) + (derivative * this.Kd) + (integralSum * this.Ki);
        double ff = Math.cos(Math.toRadians(reference / ticks_in_degree)) * f;

        return output + ff;
    }

    public void setPID(double p, double i, double d, double f) {
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
        this.f = f;
    }

    public double getKp() {
        return this.Kp;
    }

    public double getKi() {
        return this.Ki;
    }

    public double getKd() {
        return this.Kd;
    }
}
