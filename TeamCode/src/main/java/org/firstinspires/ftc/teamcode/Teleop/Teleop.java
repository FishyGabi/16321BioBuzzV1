package org.firstinspires.ftc.teamcode.Teleop;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Teleop extends LinearOpMode {
    //DEFINE

    //motors
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FIntake;
    private DcMotor BIntake;
    private DcMotorEx LShooter;
    private DcMotorEx RShooter;

    //servos
    private Servo LBlocker;
    private Servo RBlocker;
    private Servo LExtender;
    private Servo RExtender;

    //variables
    private final double SPEED = 1;

    static double integralSum = 0;
    public static double Kd = 0;
    public static double Kf = 0.000425;
    public static double Ki = 0;
    public static double Kp = 0.004;
    public static double reference = 0;
    static ElapsedTime timer = new ElapsedTime();
    static double lastError = 0;

    private final double BLOCKER_CLOSE = .5;
    private final double BLOCKER_OPEN = .7;
    private double blockerPos = BLOCKER_CLOSE;
    private final double EXTENDER_UP = .5;
    private final double EXTENDER_DOWN = .3;
    private double extenderPos = EXTENDER_UP;

    private double intakePower = 0;

    private final double TRIGGER_DEADZONE = .3;




    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");

        //HARDWARE MAP

        //motors
        FR = hardwareMap.get(DcMotor.class, "fr");
        FL = hardwareMap.get(DcMotor.class, "fl");
        BR = hardwareMap.get(DcMotor.class, "br");
        BL = hardwareMap.get(DcMotor.class, "bl");

        FIntake = hardwareMap.get(DcMotor.class, "FIntake");
        BIntake = hardwareMap.get(DcMotor.class, "BIntake");
        LShooter = hardwareMap.get(DcMotorEx.class, "LShooter");
        RShooter = hardwareMap.get(DcMotorEx.class, "RShooter");

        //servos
        LBlocker = hardwareMap.get(Servo.class, "LBlocker");
        RBlocker = hardwareMap.get(Servo.class, "RBlocker");

        LExtender = hardwareMap.get(Servo.class, "LExtender");
        RExtender = hardwareMap.get(Servo.class, "RExtender");

        //ACTUATOR DIRECTIONS

        //motors
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        BIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        RShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        //servos
        LBlocker.setDirection(Servo.Direction.REVERSE);

        LExtender.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            //ACTIVE VARIABLES

            //gamepad
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * -1.1;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            //pid
            double velocity = LShooter.getVelocity();
            double power = PIDControl(reference, velocity);


            //INPUT
            if (gamepad2.right_bumper){
                reference += 100;
            }

            if (gamepad2.left_bumper){
                reference -= 100;
            }

            if (gamepad2.dpad_down){
                reference = 0;
            }

            if (gamepad2.dpad_up){
                reference = 2000;
            }

            if (gamepad2.right_bumper){
                extenderPos = EXTENDER_DOWN;
                intakePower = 1;
            }

            if (gamepad2.left_bumper){
                extenderPos = EXTENDER_UP;
                intakePower = 0;
            }

            if (gamepad2.right_trigger > TRIGGER_DEADZONE){
                blockerPos = BLOCKER_OPEN;
                intakePower = 1;
            }

            if (gamepad2.left_trigger > TRIGGER_DEADZONE){
                blockerPos = BLOCKER_CLOSE;
                intakePower = 0;
            }

            //MOTOR SET

            //drive
            FL.setPower(frontLeftPower * SPEED);
            BL.setPower(backLeftPower * SPEED);
            FR.setPower(frontRightPower * SPEED);
            BR.setPower(backRightPower * SPEED);

            //pid
            LShooter.setPower(power);
            RShooter.setPower(power);

            //intake
            FIntake.setPower(intakePower);
            BIntake.setPower(intakePower);

            //servos
            RBlocker.setPosition(blockerPos);
            LBlocker.setPosition(blockerPos);

            LExtender.setPosition(extenderPos);
            RExtender.setPosition(extenderPos);


            telemetry.update();
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