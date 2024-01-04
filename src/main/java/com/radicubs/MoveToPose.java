package com.radicubs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.SplineParameterizer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import static java.lang.Math.atan;

public class MoveToPose extends PathFollowingCommand {
    private Trajectory trajectory;

    public <T extends SubsystemBase & HolonomicPathFollower>
    MoveToPose(T chassis, Pose2d endPose, TrajectoryConstants constants) {
        super(chassis, endPose, constants);
    }

    @Override
    public void initialize() {
        double xDiff = endPose.getX() - chassis.getRobotPose().getX();
        double yDiff = endPose.getY() - chassis.getRobotPose().getY();
        double hypoAngle = Units.radiansToDegrees(atan(yDiff / xDiff));
        SmartDashboard.putString("Status", "Running");
        timer.start();
        try {
            trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(chassis.getRobotPose().getTranslation(), Rotation2d.fromDegrees(hypoAngle)),
                    List.of(),
                    new Pose2d(endPose.getTranslation(), Rotation2d.fromDegrees(hypoAngle)),
                    new TrajectoryConfig(constants.maxPathSpeed(), constants.maxPathAcceleration()));
        } catch (SplineParameterizer.MalformedSplineException e) {
            System.out.println("Unable to generate straight trajectory to point");
            System.out.println("This may be because the robot is already at the endpoint");
            System.out.println(e);
            this.cancel();
        }
        chassis.displayTrajectory(trajectory);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = controller.calculate(chassis.getRobotPose(), trajectory.sample(timer.get()), endPose.getRotation());
        chassis.driveFromRobotOrientedChassisSpeeds(speeds);
    }
}

