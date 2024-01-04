package com.radicubs;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FollowTrajectory extends PathFollowingCommand {
    private final boolean followTrajectoryHeading;
    private final Trajectory trajectory;

    public <T extends SubsystemBase & HolonomicPathFollower>
    FollowTrajectory(T chassis, Trajectory trajectory, boolean followTrajectoryHeading, TrajectoryConstants constants) {
        super(chassis, trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters, constants);

        this.followTrajectoryHeading = followTrajectoryHeading;
        this.trajectory = trajectory;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        timer.start();
        chassis.displayTrajectory(trajectory);
        SmartDashboard.putString("Status", "Running");
    }

    @Override
    public void execute() {
        Trajectory.State state = trajectory.sample(timer.get());
        ChassisSpeeds speeds = controller.calculate(chassis.getRobotPose(), state, followTrajectoryHeading ? state.poseMeters.getRotation() : endPose.getRotation());
        SmartDashboard.putNumber("PID Target X", controller.getXController().getSetpoint());
        SmartDashboard.putNumber("PID Target Y", controller.getYController().getSetpoint());
        SmartDashboard.putNumber("PID Target Angle", controller.getThetaController().getSetpoint().position);
        chassis.driveFromRobotOrientedChassisSpeeds(speeds);
    }
}
