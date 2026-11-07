package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import com.pedropathing.follower.Follower;

@Autonomous
public class BlueAuto extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(24, 24, 0);
    private final Pose park = p.of(48, 48, 90);

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {
    }
}