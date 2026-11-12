package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Constants;
import org.firstinspires.ftc.teamcode.OpModeStorage;

@TeleOp
public class PedroTeleop extends OpMode {
    //DEFINE

    //motors
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
    private final double BLOCKER_OPEN = .15;
    private double blockerPos = BLOCKER_CLOSE;
    private final double EXTENDER_UP = .5;
    private final double EXTENDER_DOWN = .65;
    private double extenderPos = EXTENDER_UP;
    private final double RAMP_DOWN = .0;
    private final double RAMP_UP = .7;
    private double rampPos = RAMP_UP;

    private double intakePower = 0;

    private final double TRIGGER_DEADZONE = .3;

    private boolean intakeRunning = false;

    private double rx;
    private double ry;
    private double rh;

    private final double RED_BOT_GX = 59;
    private final double RED_BOT_GY = 52;

    private final double RED_TOP_GX = 59;
    private final double RED_TOP_GY = 92;

    private final double BLUE_BOT_GX = 85;
    private final double BLUE_BOT_GY = 52;

    private final double BLUE_TOP_GX = 85;
    private final double BLUE_TOP_GY = 92;
    private static double aimKp = 1.5;

    private double gx;
    private double gy;

    private Follower follower;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        follower = Constants.create(hardwareMap);

        //HARDWARE MAP

        //motors

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

        BIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        RShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        //servos
        LBlocker.setDirection(Servo.Direction.REVERSE);

        LExtender.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void start(){
        if (OpModeStorage.red) {
            follower.setPose(OpModeStorage.redAutonomousEndPose);
        } else {
            follower.setPose(OpModeStorage.blueAutonomousEndPose);
        }
        follower.update();
    }

    @Override
    public void loop() {
        //Pedro Stuff
        double forward = -gamepad1.left_stick_y * speed;
        double lateral = -gamepad1.left_stick_x * speed;
        double turn = -gamepad1.right_stick_x * speed;


        //relocalize

        if (gamepad1.start && gamepad1.x){
            //FIXME: Find correct Pose
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            follower.setPose(cornerPose);
        }

        follower.update();

        Pose robotPose = follower.pose();

        rx = robotPose.x();
        ry = robotPose.y();
        rh = robotPose.heading();

        telemetry.addData("Robot X", rx);
        telemetry.addData("Robot Y", ry);
        telemetry.addData("Robot Heading", rh);

        double velocity = LShooter.getVelocity();
        double power = PIDControl(reference, velocity);

        telemetry.addData("Speed", velocity);

        //Auto Aim

        if (OpModeStorage.red){
            if (ry > 72){
                gx = RED_TOP_GX;
                gy = RED_TOP_GY;
            }else {
                gx = RED_BOT_GX;
                gy = RED_BOT_GY;
            }
        }else {
            if (ry > 72){
                gx = BLUE_TOP_GX;
                gy = BLUE_TOP_GY;
            }else {
                gx = BLUE_BOT_GX;
                gy = BLUE_BOT_GY;
            }
        }

        if (gamepad1.a && !gamepad1.start){
            double angle = Math.atan2(gy-ry, gx-rx);
            double error = angleWrap(angle - rh);

            turn = Math.abs(error) < Math.toRadians(2)
                    ? 0
                    :error*aimKp;
        }

        ManualDrive.driveOrHold(follower, forward, lateral, turn);

        //INPUT

        if (gamepad2.dpad_down){
            reference = 0;
        }

        if (gamepad2.dpad_up){
            reference = 950;
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

        if (gamepad1.right_trigger > TRIGGER_DEADZONE){
            rampPos = RAMP_DOWN;
        }

        if (gamepad1.left_trigger > TRIGGER_DEADZONE){
            rampPos = RAMP_UP;
        }

        if (gamepad1.left_bumper){
            speed = SLOW_SPEED;
        }

        if (gamepad1.right_bumper){
            speed = FAST_SPEED;
        }

        //MOTOR SET

        //drive

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

    private static double angleWrap(double angle){
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI)  angle += 2 * Math.PI;
        return angle;
    }


}
