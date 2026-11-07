package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Auto extends LinearOpMode {

    private DcMotor FR;
    private DcMotor FL;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FIntake;
    private DcMotor BIntake;
    private DcMotorEx LShooter;
    private DcMotorEx RShooter;

    private Servo LBlocker;
    private Servo RBlocker;
    private Servo LExtender;
    private Servo RExtender;
    private Servo LRamp;
    private Servo RRamp;

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
    private final double RAMP_DOWN = .0;
    private final double RAMP_UP = .7;
    private double rampPos = RAMP_UP;

    private double intakePower = 0;
    private int state = 0;
    ElapsedTime newTimer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");


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

        waitForStart();

        LBlocker.setPosition(BLOCKER_CLOSE);
        RBlocker.setPosition(BLOCKER_CLOSE);

        LExtender.setPosition(EXTENDER_UP);
        LExtender.setPosition(EXTENDER_DOWN);

        LRamp.setPosition(RAMP_UP);
        RRamp.setPosition(RAMP_UP);

        int timer = 0;

        while (opModeIsActive()){
            telemetry.addData("Speed", LShooter.getVelocity());

            double velocity = LShooter.getVelocity();
            double power = PIDControl(reference, velocity);

            LShooter.setPower(power);
            RShooter.setPower(power);

            switch (state){
                case 0:
                    reference = 1550;
                    if (velocity > 1500){
                        state++;
                        timer = 0;
                    }
                    break;
                case 1:
                    blockerPos = BLOCKER_OPEN;
                    intakePower = 1;
                    timer++;
                    if (timer >100000){
                        state++;
                        timer = 0;
                    }
                    break;
                case 2:
                    intakePower = 0;
                    FR.setPower(-1);
                    FL.setPower(-1);
                    BR.setPower(-1);
                    BL.setPower(-1);
                    timer++;
                    if (timer > 10000){
                        FR.setPower(0);
                        FL.setPower(0);
                        BR.setPower(0);
                        BL.setPower(0);
                        requestOpModeStop();
                        state++;
                    }
                    break;
            }

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
