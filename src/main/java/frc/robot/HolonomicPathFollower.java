package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class HolonomicPathFollower extends SubsystemBase{
    public abstract void driveFromFieldOrientedChassisSpeeds(ChassisSpeeds fieldRelativeSpeeds);
    public abstract void driveFromRobotOrientedChassisSpeeds(ChassisSpeeds robotOrientedSpeeds);
    public abstract void displayTrajectory(Trajectory trajectory);
    public abstract Pose2d getRobotPose();
}