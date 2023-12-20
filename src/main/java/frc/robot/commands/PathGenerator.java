package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.SplineParameterizer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.HolonomicPathFollower;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;


public class PathGenerator {


    public static <T extends SubsystemBase & HolonomicPathFollower>
    CommandBase fromPathweaverJSON(T chassis, TrajectoryConstants constants, boolean followTrajectoryHeading,
                                   String jsonFilePath) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(jsonFilePath);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            return new FollowTrajectory(chassis, trajectory, followTrajectoryHeading, constants);
        } catch (IOException e) {
            System.out.println("File \"" + jsonFilePath + "\" not found");
            return new InstantCommand();
        } catch (SplineParameterizer.MalformedSplineException e) {
            System.out.println("Unable to generate trajectory from Pathweaver file");
            System.out.println(e);
            return new InstantCommand();
        }
    }

    public static <T extends SubsystemBase & HolonomicPathFollower>
    CommandBase fromSplinePoints(T chassis, TrajectoryConstants constants, boolean followTrajectoryHeading,
                                 Pose2d... waypoints) {
        try {
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(waypoints),
                    new TrajectoryConfig(constants.maxPathSpeed(), constants.maxPathAcceleration()));
            return new FollowTrajectory(chassis, trajectory, followTrajectoryHeading, constants);
        } catch (SplineParameterizer.MalformedSplineException e) {
            System.out.println("Unable to generate spline trajectory");
            System.out.println("This is often caused by listing the same point twice");
            System.out.println(e);
            return new InstantCommand();
        }
    }

    public static <T extends SubsystemBase & HolonomicPathFollower>
    CommandBase fromSimplifiedSplinePoints(T chassis, TrajectoryConstants constants, boolean followTrajectoryHeading,
                                           Pose2d startingPose, Pose2d endingPose, Translation2d... midpoints) {
        try {
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                    startingPose,
                    Arrays.asList(midpoints),
                    endingPose,
                    new TrajectoryConfig(constants.maxPathSpeed(), constants.maxPathAcceleration()));
            return new FollowTrajectory(chassis, trajectory, followTrajectoryHeading, constants);
        } catch (Exception e) {
            System.out.println("Unable to generate simplified spline trajectory");
            System.out.println("This is often caused by listing the same point twice");
            System.out.println(e);
            return new InstantCommand();
        }
    }

    public static <T extends SubsystemBase & HolonomicPathFollower>
    SequentialCommandGroup fromStraightPoints(T chassis, TrajectoryConstants constants, Pose2d... waypoints) {
        Command[] commands = new Command[waypoints.length];

        for (int i = 0; i < waypoints.length; i++) {
            commands[i] = new MoveToPose(chassis, waypoints[i], constants);
        }
        return new SequentialCommandGroup(commands);
    }
}
