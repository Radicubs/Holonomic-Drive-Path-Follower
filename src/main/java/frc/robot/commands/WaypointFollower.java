package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HolonomicChassisSim;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;

public class WaypointFollower extends CommandBase {
    public WaypointFollower(HolonomicChassisSim chassisSim, Translation2d ...waypoints){
        this(chassisSim, new Rotation2d(), new Rotation2d(), waypoints);
    }
    public WaypointFollower(HolonomicChassisSim chassisSim, Rotation2d startingRot, Rotation2d endingRot, Translation2d ...waypoints){
        if(waypoints.length < 2){
            System.out.println("bruhhhh wha da heeeeeelllllll aint no way blud tryna use 1 waypoint to generate a trajectory you goofy ahh go back to cs 1");
            this.cancel();
        }
        // Generate Trajectory
        Pose2d startingPose = new Pose2d(waypoints[0], startingRot);
        Pose2d endingPose = new Pose2d(waypoints[waypoints.length - 1], endingRot);
        List<Translation2d> midpoints = Arrays.asList(Arrays.copyOfRange(waypoints, 1, waypoints.length - 1));
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                startingPose,
                midpoints,
                endingPose,
                new TrajectoryConfig(Constants.Simulation.MAX_AXIS_SPEED, Constants.Simulation.MAX_ACCELERATION));

        //Display Trajectory
        chassisSim.displayTrajectory(trajectory);
        addRequirements(chassisSim);
    }
}
