package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;

/**
 * Subsystem wrapper for Pedro Pathing's Follower.
 * Automatically updates follower each scheduler tick and
 * provides a method to reset odometry.
 */
public class FollowerSubsystem extends SubsystemBase {
    private final Follower follower;

    /**
     * Construct and initialize the Pedro Follower.
     * @param hardwareMap FTC hardware map
     * @param fConstants  Pathing constants class
     * @param lConstants  Localization constants class
     * @param startPose   Initial robot pose
     */
    public FollowerSubsystem(HardwareMap hardwareMap,
                             Class<?> fConstants,
                             Class<?> lConstants,
                             Pose startPose) {
        Constants.setConstants(fConstants, lConstants);
        follower = new Follower(hardwareMap, fConstants, lConstants);
        follower.setStartingPose(startPose);
    }

    /**
     * Reset the follower's internal pose to a new starting pose.
     * @param newPose the pose to reset to
     */
    public void setStartingPose(Pose newPose) {
        follower.setStartingPose(newPose);
    }

    /**
     * Expose current pose for telemetry or logic.
     */
    public Pose getPose() {
        return follower.getPose();
    }

    @Override
    public void periodic() {
        // Called once per scheduler run to update odometry and control
        follower.update();
    }
}
