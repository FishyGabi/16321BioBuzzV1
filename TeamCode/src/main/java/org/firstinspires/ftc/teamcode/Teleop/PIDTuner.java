package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;


@Configurable
@TeleOp
public class PIDTuner extends LinearOpMode {
    private TelemetryManager panelsTelemetry;
    static double integralSum = 0;
    public static double Kd = 0;
    public static double Kf = 0.000425;
    public static double Ki = 0;
    public static double Kp = 0.004;
    public static double reference = 1000;
    static ElapsedTime timer = new ElapsedTime();
    static double lastError = 0;

    private DcMotorEx LShooter;
    private DcMotorEx RShooter;
    private DcMotor FIntake;
    private DcMotor BIntake;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");

        LShooter = hardwareMap.get(DcMotorEx .class, "LShooter");
        RShooter = hardwareMap.get(DcMotorEx.class, "RShooter");

        FIntake = hardwareMap.get(DcMotor.class, "FIntake");
        BIntake = hardwareMap.get(DcMotor.class, "BIntake");

        RShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        BIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();



        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Speed", LShooter.getVelocity());

            double velocity = LShooter.getVelocity();
            double power = PIDControl(reference, velocity);

            if (gamepad1.right_bumper) {
                BIntake.setPower(-1);
                FIntake.setPower(-1);
            }
            if (gamepad1.left_bumper) {
                BIntake.setPower(0);
                FIntake.setPower(0);
            }

            LShooter.setPower(power);
            RShooter.setPower(power);
            panelsTelemetry.addData("Reference", reference);
            panelsTelemetry.addData("Speed", velocity);
            panelsTelemetry.update(telemetry);
            telemetry.update();

        }
    }

    public static double PIDControl(double reference, double state){
        double error = reference - state;
        double dt = timer.seconds();
        timer.reset();
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
