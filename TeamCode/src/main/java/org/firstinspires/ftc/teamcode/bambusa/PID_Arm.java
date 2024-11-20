package org.firstinspires.ftc.teamcode.bambusa;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PID_Arm extends OpMode {

    private PIDController controller;

    public static double p = 0.01, i = 0.2, d = 0.0011;
    public static double f = 0.5;

    public static int target = 0;

    private final double ticks_in_degree = 300 / 90.0;

    private DcMotorEx slideChain;

    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slideChain = hardwareMap.get(DcMotorEx.class, "slideChain");
    }


    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = slideChain.getCurrentPosition();
        double pid = controller.calculate(armPos, target);

        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        slideChain.setPower(power);

        telemetry.addData("pos: ", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
