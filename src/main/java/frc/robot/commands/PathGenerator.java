package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.SplineParameterizer;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.HolonomicPathFollower;
import frc.robot.Constants.TrajectoryFollower;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;


public class PathGenerator {
    private static HolonomicDriveController controller = new HolonomicDriveController(
            new PIDController(TrajectoryFollower.xControllerKP, TrajectoryFollower.xControllerKI, TrajectoryFollower.xControllerKD),
            new PIDController(TrajectoryFollower.yControllerKP, TrajectoryFollower.yControllerKI, TrajectoryFollower.yControllerKD),
            new ProfiledPIDController(TrajectoryFollower.thetaControllerKP, TrajectoryFollower.thetaControllerKI, TrajectoryFollower.thetaControllerKD,
                    new TrapezoidProfile.Constraints(TrajectoryFollower.MAX_PATH_ANGULAR_SPEED, TrajectoryFollower.MAX_PATH_ANGULAR_ACCELERATION)));

    private static Pose2d tolerance = new Pose2d(new Translation2d(TrajectoryFollower.xToleranceMeters, TrajectoryFollower.yToleranceMeters),
            Rotation2d.fromDegrees(TrajectoryFollower.rotToleranceDeg));

    public static <T extends SubsystemBase & HolonomicPathFollower> CommandBase fromPathweaverJSON(T chassis, boolean followTrajectoryHeading, String jsonFilePath){
        try{
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(jsonFilePath);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            return new FollowTrajectory(chassis, trajectory, controller, tolerance, followTrajectoryHeading);
        }
        catch(IOException e){
            System.out.println("File \"" + jsonFilePath + "\" not found");
            return new InstantCommand();
        }
        catch (SplineParameterizer.MalformedSplineException e) {
            System.out.println("Unable to generate trajectory from Pathweaver file");
            System.out.println(e);
            return new InstantCommand();
        }
    }

    public static <T extends SubsystemBase & HolonomicPathFollower> CommandBase fromSplinePoints(T chassis, boolean followTrajectoryHeading, Pose2d ...waypoints){
        try {
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(waypoints),
                    new TrajectoryConfig(TrajectoryFollower.MAX_PATH_SPEED, TrajectoryFollower.MAX_PATH_ACCELERATION));
            return new FollowTrajectory(chassis, trajectory, controller, tolerance, followTrajectoryHeading);
        } catch (SplineParameterizer.MalformedSplineException e) {
            System.out.println("Unable to generate spline trajectory");
            System.out.println("This is often caused by listing the same point twice");
            System.out.println(e);
            return new InstantCommand();
        }
    }

    public static <T extends SubsystemBase & HolonomicPathFollower> CommandBase fromSimplifiedSplinePoints(T chassis, boolean followTrajectoryHeading, Pose2d startingPose, Pose2d endingPose, Translation2d ...midpoints){
        try {
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                    startingPose,
                    Arrays.asList(midpoints),
                    endingPose,
                    new TrajectoryConfig(TrajectoryFollower.MAX_PATH_SPEED, TrajectoryFollower.MAX_PATH_ACCELERATION));
            return new FollowTrajectory(chassis, trajectory, controller, tolerance, followTrajectoryHeading);
        } catch (Exception e) {
            System.out.println("Unable to generate simplified spline trajectory");
            System.out.println("This is often caused by listing the same point twice");
            System.out.println(e);
            return new InstantCommand();
        }
    }

    public static <T extends SubsystemBase & HolonomicPathFollower> CommandBase fromStraightPoints(T chassis, Pose2d ...waypoints){
        Command[] commands = new Command[waypoints.length];
        for(int i=0; i< waypoints.length; i++){
            commands[i] = new MoveToPose(chassis, waypoints[i], controller, tolerance);
        }
        return new SequentialCommandGroup(commands);
    }
}
