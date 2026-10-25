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
    private Servo LRamp;
    private Servo RRamp;

    //variables
    private double speed = 1;

    private final double FAST_SPEED = 1;
    private final double SLOW_SPEED = .5;

    static double integralSum = 0;
    public static double Kd = 0;
    public static double Kf = 0.000425;
    public static double Ki = 0;
    public static double Kp = 0.004;
    public static double reference = 0;
    static ElapsedTime timer = new ElapsedTime();
    static double lastError = 0;

    private final double BLOCKER_CLOSE = .3;
    private final double BLOCKER_OPEN = .2;
    private double blockerPos = BLOCKER_CLOSE;
    private final double EXTENDER_UP = .5;
    private final double EXTENDER_DOWN = .65;
    private double extenderPos = EXTENDER_UP;
    private final double RAMP_DOWN = .3;
    private final double RAMP_UP = .3;
    private double rampPos = RAMP_UP;

    private double intakePower = 0;

    private final double TRIGGER_DEADZONE = .3;

    private boolean intakeRunning = false;




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

        RRamp = hardwareMap.get(Servo.class, "RRamp");
        LRamp = hardwareMap.get(Servo.class, "LRamp");

        //ACTUATOR DIRECTIONS

        //motors
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        RShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        //servos
        LBlocker.setDirection(Servo.Direction.REVERSE);

        LExtender.setDirection(Servo.Direction.REVERSE);

        LRamp.setDirection(Servo.Direction.REVERSE);

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

            telemetry.addData("Speed", reference);

            //INPUT
            if (gamepad2.dpad_right){
                reference += 1;
            }

            if (gamepad2.dpad_left){
                reference -= 1;
            }

            if (gamepad2.dpad_down){
                reference = 0;
            }

            if (gamepad2.dpad_up){
                reference = 1500;
            }

            if (gamepad2.right_bumper){
                extenderPos = EXTENDER_DOWN;
                blockerPos = BLOCKER_CLOSE;

                intakePower = 1;

                intakeRunning = true;
            }

            if (gamepad2.left_bumper){
                extenderPos = EXTENDER_UP;
                blockerPos = BLOCKER_CLOSE;

                intakePower = 0;

                intakeRunning = false;
            }

            if (gamepad2.right_trigger > TRIGGER_DEADZONE){
                blockerPos = BLOCKER_OPEN;
                intakePower = 1;
                intakeRunning = false;
            }

            if (gamepad2.left_trigger > TRIGGER_DEADZONE){
                blockerPos = BLOCKER_CLOSE;
                intakePower = -1;
                intakeRunning = false;
            }

            if (!intakeRunning && gamepad2.right_trigger < TRIGGER_DEADZONE && gamepad2.left_trigger < TRIGGER_DEADZONE){
                blockerPos = BLOCKER_CLOSE;
                intakePower = 0;
            }

            if (gamepad1.right_bumper){
                rampPos = RAMP_DOWN;
            }

            if (gamepad1.left_bumper){
                rampPos = RAMP_UP;
            }

            if (gamepad1.left_trigger > TRIGGER_DEADZONE){
                speed = SLOW_SPEED;
            }

            if (gamepad1.left_trigger > TRIGGER_DEADZONE){
                speed = FAST_SPEED;
            }


            //MOTOR SET

            //drive
            FL.setPower(frontLeftPower * speed);
            BL.setPower(backLeftPower * speed);
            FR.setPower(frontRightPower * speed);
            BR.setPower(backRightPower * speed);

            //pid
            LShooter.setPower(power);
            RShooter.setPower(power);

            //intake
            FIntake.setPower(-intakePower);
            BIntake.setPower(-intakePower);

            //servos
            RBlocker.setPosition(blockerPos);
            LBlocker.setPosition(blockerPos);

            LExtender.setPosition(extenderPos);
            RExtender.setPosition(extenderPos);

            RRamp.setPosition(rampPos);
            LRamp.setPosition(rampPos);


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