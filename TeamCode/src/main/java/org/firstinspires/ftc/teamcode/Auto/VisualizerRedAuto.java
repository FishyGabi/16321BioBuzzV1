package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.OpModeStorage;


@Autonomous
public class VisualizerRedAuto extends OpMode {
    public Follower follower; // Pathing follower instance
    private final PoseFactory p = PoseFactory.degrees();
    private int pathState; // Current autonomous path state (state machine)
    private ElapsedTime pathTimer; // Timer for path state machine
    private Paths paths; // Paths defined in the Paths class
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
    private final int SHOOT_TIME = 250;

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);
        follower.setPose(p.of(60.000, 8.500, 90.000));

        pathTimer = new ElapsedTime();
        paths = new Paths(follower);

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
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        double velocity = LShooter.getVelocity();
        double power = PIDControl(reference, velocity);

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


    public static class Paths {
        private static final PoseFactory p = PoseFactory.degrees();
        public Path FlowerPos1;
        public Path FlowerIntake1;
        public Path ShootPos1;
        public Path FlowerPos2;
        public Path FlowerIntake2;
        public Path ShootPos2;
        public Path Park;

        public Paths(Follower follower) {
            FlowerPos1 = line(
                    p.of(60.000, 8.500, 0.0),
                    p.of(24.530, 48.000, 0.0)
            ).linear(Math.toRadians(90), Math.toRadians(180));

            FlowerIntake1 = line(
                    p.of(24.530, 48.000, 0.0),
                    p.of(18.000, 48.000, 0.0)
            ).linear(Math.toRadians(180), Math.toRadians(180));

            ShootPos1 = curve(
                    p.of(18.000, 48.000, 0.0),
                    p.of(18.000, 106.000, 0.0),
                    p.of(60.000, 107.000, 0.0),
                    p.of(60.000, 135.500, 0.0)
            ).linear(Math.toRadians(180), Math.toRadians(-90));

            FlowerPos2 = curve(
                    p.of(60.000, 135.500, 0.0),
                    p.of(60.000, 128.255, 0.0),
                    p.of(48.000, 120.000, 0.0)
            ).linear(Math.toRadians(-90), Math.toRadians(90));

            FlowerIntake2 = line(
                    p.of(48.000, 120.000, 0.0),
                    p.of(48.000, 130.000, 0.0)
            ).linear(Math.toRadians(90), Math.toRadians(90));

            ShootPos2 = curve(
                    p.of(48.000, 130.000, 0.0),
                    p.of(56.652, 109.109, 0.0),
                    p.of(60.000, 135.500, 0.0)
            ).heading(Interpolator.longLinear(Math.toRadians(90), Math.toRadians(-90)));

            Park = curve(
                    p.of(60.000, 135.500, 0.0),
                    p.of(60.000, 118.573, 0.0),
                    p.of(14.000, 118.573, 0.0)
            ).tangent();
        }



    }





    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                reference = 950;
                setPathState(1);
                break;
            case 1:
                if (pathTimer.milliseconds() > 1000) {
                    blockerPos = BLOCKER_OPEN;
                    intakePower = 1;
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.milliseconds() > SHOOT_TIME) {
                    blockerPos = BLOCKER_CLOSE;
                    reference = 0;
                    follower.follow(paths.FlowerPos1);
                    extenderPos = EXTENDER_DOWN;
                    rampPos = RAMP_DOWN;

                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4:
                follower.follow(paths.FlowerIntake1);
                setPathState(5);
                break;
            case 5:
                if(!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.milliseconds() > 2000){
                    intakePower = 0;
                    reference = 950;
                    setPathState(7);
                }
                break;
            case 7:
                follower.follow(paths.ShootPos1);
                setPathState(8);
                break;
            case 8:
                if(!follower.isBusy()) {
                    setPathState(9);
                    blockerPos=BLOCKER_OPEN;
                    intakePower = 1;
                }
                if (pathTimer.milliseconds() > 1000){
                    rampPos = RAMP_UP;
                }
                break;

            case 9:
                if (pathTimer.milliseconds() > SHOOT_TIME){
                    intakePower = 0;
                    blockerPos = BLOCKER_CLOSE;
                    setPathState(10);
                }
                break;
            case 10:
                follower.follow(paths.FlowerPos2);
                rampPos = RAMP_DOWN;
                setPathState(11);
                break;
            case 11:
                if(!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case 12:
                follower.follow(paths.FlowerIntake2);
                intakePower = 1;
                setPathState(13);
                break;
            case 13:
                if(!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case 14:
                if(pathTimer.milliseconds() > 2000){
                    setPathState(15);
                    intakePower = 0;
                }
                break;
            case 15:
                follower.follow(paths.ShootPos2);
                setPathState(16);
                break;
            case 16:
                if(!follower.isBusy()) {
                    setPathState(17);
                    blockerPos = BLOCKER_OPEN;
                    intakePower = 1;
                    rampPos = RAMP_UP;
                }
                break;
            case 17:
                if (pathTimer.milliseconds() > SHOOT_TIME){
                    intakePower = 0;
                    blockerPos = BLOCKER_CLOSE;
                    setPathState(18);

                }
            case 18:
                follower.follow(paths.Park);
                setPathState(19);
                break;
            case 19:
                if(!follower.isBusy()) {
                    setPathState(20);
                }
                break;
            case 20:
                requestOpModeStop();
                pathState = -1;
                break;
        }
        return pathState;
    }

    @Override
    public void stop() {
        OpModeStorage.redAutonomousEndPose = follower.pose(); //saves your position in that file
        OpModeStorage.red = true;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
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
    