package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class PIDTuner extends LinearOpMode {
    static double integralSum = 0;
    public static double Kd = 0;
    public static double Kf = 0.000425;
    public static double Ki = 0;
    public static double Kp = 0.004;
    public static double reference = 0;
    static ElapsedTime timer = new ElapsedTime();
    static double lastError = 0;

    private DcMotorEx LShooter;
    private DcMotorEx RShooter;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");

        LShooter = hardwareMap.get(DcMotorEx .class, "LShooter");
        RShooter = hardwareMap.get(DcMotorEx.class, "RShooter");

        waitForStart();

        while(opModeIsActive()){

            double velocity = LShooter.getVelocity();
            double power = PIDControl(reference, velocity);

        }
    }

    public static double PIDControl(double reference, double state){
        double error = reference - state;
        double dt = timer.seconds();
        if (dt <= 0 || dt > 0.5) dt = 0.020;

        double MAX_INTEGRAL = 0.25 / Ki;

        if (Ki > 0) {
            integralSum += error * dt;
            integralSum = Math.max(-MAX_INTEGRAL, Math.min(integralSum, MAX_INTEGRAL));
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (error * Kp)
                + (derivative * Kd)
                + (integralSum * Ki)
                + (reference * Kf);

        output = Math.max(-1, Math.min(output, 1));

        return output;
    }
}
