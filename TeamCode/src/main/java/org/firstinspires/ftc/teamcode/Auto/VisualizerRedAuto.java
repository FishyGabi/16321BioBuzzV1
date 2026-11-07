package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import com.pedropathing.follower.Follower;
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

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);
        follower.setPose(p.of(60.000, 8.500, 90.000));

        pathTimer = new ElapsedTime();
        paths = new Paths(follower);

    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();
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
                    p.of(14.000, 48.000, 0.0)
            ).linear(Math.toRadians(180), Math.toRadians(180));

            ShootPos1 = curve(
                    p.of(14.000, 48.000, 0.0),
                    p.of(14.000, 106.000, 0.0),
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
                follower.follow(paths.FlowerPos1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case 2:
                follower.follow(paths.FlowerIntake1);
                setPathState(3);
                break;
            case 3:
                if(!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4:
                follower.follow(paths.ShootPos1);
                setPathState(5);
                break;
            case 5:
                if(!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                follower.follow(paths.FlowerPos2);
                setPathState(7);
                break;
            case 7:
                if(!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                follower.follow(paths.FlowerIntake2);
                setPathState(9);
                break;
            case 9:
                if(!follower.isBusy()) {
                    setPathState(10);
                }
                break;
            case 10:
                follower.follow(paths.ShootPos2);
                setPathState(11);
                break;
            case 11:
                if(!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case 12:
                follower.follow(paths.Park);
                setPathState(13);
                break;
            case 13:
                if(!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case 14:
                requestOpModeStop();
                pathState = -1;
                break;
        }
        return pathState;
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }
}
    