package com.radicubs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

public interface HolonomicPathFollower {
    void driveFromFieldOrientedChassisSpeeds(ChassisSpeeds fieldRelativeSpeeds);

    void driveFromRobotOrientedChassisSpeeds(ChassisSpeeds robotOrientedSpeeds);

    Pose2d getRobotPose();

    default void displayTrajectory(Trajectory trajectory) {
    }
}