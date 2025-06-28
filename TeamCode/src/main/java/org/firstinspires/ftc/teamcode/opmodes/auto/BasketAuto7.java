// File: teamcode/opmodes/auto/BasketAuto7.java
package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.collection.ArraySet;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Test 5 Spec", group = "Autonomous")
public class BasketAuto7 extends CommandOpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private PathChain path1, path2;
    private final Pose startPose = new Pose(7.722, 103.304, 0);

    @Override
    public void initialize() {
        // SET UP PEDRO PATHING
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        // BUILD PATHS
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(7.722, 103.304, Point.CARTESIAN),
                        new Point(24.209,  95.313, Point.CARTESIAN),
                        new Point(7.000,   90.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(18.100, 125.635, Point.CARTESIAN),
                        new Point(18.243, 120.670, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        // SCHEDULE AUTO
        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new SequentialCommandGroup(
                                // reset pose on the road-runner pose tracker
                                new InstantCommand(() -> follower.setStartingPose(startPose)),
                                // run paths sequentially
                                new FollowPathCommand(follower, path1, false)
//                                new FollowPathCommand(follower, path2, false)
                        )
                )
        );
    }

    @Override
    public void run() {
        // run the scheduled commands and update follower
        super.run();
        follower.update();
    }
}
