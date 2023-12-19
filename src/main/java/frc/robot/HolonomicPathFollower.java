package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface HolonomicPathFollower{
    void driveFromFieldOrientedChassisSpeeds(ChassisSpeeds fieldRelativeSpeeds);
    void driveFromRobotOrientedChassisSpeeds(ChassisSpeeds robotOrientedSpeeds);
    Pose2d getRobotPose();
    default void displayTrajectory(Trajectory trajectory){}
}