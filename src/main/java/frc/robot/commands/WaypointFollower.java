package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HolonomicChassisSim;

import java.util.Arrays;
import java.util.List;

public class WaypointFollower extends CommandBase {

    private  HolonomicChassisSim chassis;
    private  Rotation2d startingRotation;
    private  Rotation2d endingRotation;
    private  Translation2d[] waypoints;

    //private final Trajectory trajectory;
    private  HolonomicDriveController controller;
    private  Timer timer;
    private  Pose2d startingPose, endingPose, tolerance;

    Trajectory trajectory = new Trajectory();

    private WaypointFollower(HolonomicChassisSim chassis, Rotation2d startingRotation, Rotation2d endingRotation) {
        this.chassis = chassis;
        this.startingRotation = startingRotation;
        this.endingRotation = endingRotation;

        addRequirements(this.chassis);

        // Create PID Controller
        controller = new HolonomicDriveController(
                new PIDController(0,0 ,0),
                new PIDController(0,0, 0),
                new ProfiledPIDController(0, 0, 0,
                        new TrapezoidProfile.Constraints(Constants.Simulation.MAX_ANGULAR_SPEED, Constants.Simulation.MAX_ANGULAR_ACCELERATION)));
        timer = new Timer();

        // Set Tolerance
        this.tolerance = new Pose2d(new Translation2d(0.03, 0.03), new Rotation2d(Units.degreesToRadians(2)));
        controller.setTolerance(tolerance);
    }

    public WaypointFollower(HolonomicChassisSim chassis, Rotation2d startingRotation, Rotation2d endingRotation, boolean spline, Translation2d... waypoints) {
        this(chassis, startingRotation, endingRotation);

        this.waypoints = waypoints;

        this.startingPose = new Pose2d(waypoints[0], startingRotation);
        this.endingPose = new Pose2d(waypoints[waypoints.length - 1], endingRotation);

        if(spline) {
            List<Translation2d> midpoints = Arrays.asList(Arrays.copyOfRange(waypoints, 1, waypoints.length - 1));
            trajectory = TrajectoryGenerator.generateTrajectory(
                    startingPose,
                    midpoints,
                    endingPose,
                    new TrajectoryConfig(Constants.Simulation.MAX_AXIS_SPEED, Constants.Simulation.MAX_ACCELERATION));
        }

        else {
            // do not spline code
        }

        chassis.displayTrajectory(trajectory);


    }

    private static Rotation2d getStartRotationFromFile(String file) {
        return null;
    }

    private static Rotation2d getEndRotationFromFile(String file) {
        return null;
    }

    public WaypointFollower(HolonomicChassisSim chassis, String file) {
        this(chassis, getStartRotationFromFile(file), getEndRotationFromFile(file));
        // fiile code here yipee
    }



//
//    public WaypointFollower(HolonomicChassisSim chassisSim, Translation2d ...waypoints) throws IOException {
//        this(chassisSim, new Rotation2d(), new Rotation2d(), waypoints);
//    }
//
//    public WaypointFollower(HolonomicChassisSim chassisSim, Rotation2d startingRot, Rotation2d endingRot, Translation2d ...waypoints) throws IOException {
//        if(waypoints.length < 2){
//            System.out.println("bruhhhh wha da heeeeeelllllll aint no way blud tryna use 1 waypoint to generate a trajectory you goofy ahh go back to cs 1");
//            this.cancel();
//        }
//        this.chassisSim = chassisSim;
//        this.startingRot = startingRot;
//        this.endingRot = endingRot;
//        this.waypoints = waypoints;
//
//        // Display Trajectory
//
//        chassisSim.displayTrajectory(trajectory);
//        addRequirements(chassisSim);
//
//        // Create PID Controller
//        controller = new HolonomicDriveController(
//                new PIDController(0,0 ,0),
//                new PIDController(0,0, 0),
//                new ProfiledPIDController(0, 0, 0,
//                        new TrapezoidProfile.Constraints(Constants.Simulation.MAX_ANGULAR_SPEED, Constants.Simulation.MAX_ANGULAR_ACCELERATION)));
//        timer = new Timer();
//
//        // Set Tolerance
//        this.tolerance = new Pose2d(new Translation2d(0.03, 0.03), new Rotation2d(Units.degreesToRadians(2)));
//        controller.setTolerance(tolerance);
//
//    }
//
//    public WaypointFollower() throws IOException {
//       this();
//        // Generate Trajectory
//        this.startingPose = new Pose2d(waypoints[0], startingRot);
//        this.endingPose = new Pose2d(waypoints[waypoints.length - 1], endingRot);
//
//        List<Translation2d> midpoints = Arrays.asList(Arrays.copyOfRange(waypoints, 1, waypoints.length - 1));
//        trajectory = TrajectoryGenerator.generateTrajectory(
//                startingPose,
//                midpoints,
//                endingPose,
//                new TrajectoryConfig(Constants.Simulation.MAX_AXIS_SPEED, Constants.Simulation.MAX_ACCELERATION));
//    }
//
//    public WaypointFollower() throws IOException {
//        this();
//        // Trajectory from Pathweaver
//
//        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
//        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
//
//        startingPose = trajectory.getInitialPose();
//        endingPose = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;
//    }


    @Override
    public void initialize() {
        timer.start();
        chassis.setRobotPose(trajectory.getInitialPose());
        SmartDashboard.putString("Status", "Running");
    }

    @Override
    public void execute() {
        Trajectory.State goal = trajectory.sample(timer.get());
        ChassisSpeeds speeds = controller.calculate(chassis.getRobotPose(), trajectory.sample(timer.get()), goal.poseMeters.getRotation());
        SmartDashboard.putNumber("PID Target X", controller.getXController().getSetpoint());
        SmartDashboard.putNumber("PID Target Y", controller.getYController().getSetpoint());
        SmartDashboard.putNumber("PID Target Angle", goal.poseMeters.getRotation().getDegrees());
        chassis.driveFromRobotOrientedChassisSpeeds(speeds, false);
    }

    @Override
    public boolean isFinished() {
        Transform2d diff = chassis.getRobotPose().minus(endingPose);
        double xDiff = diff.getX();
        double yDiff = diff.getY();
        double rotDifDeg = diff.getRotation().getDegrees();

        SmartDashboard.putNumber("X Diff to End Pose", xDiff);
        SmartDashboard.putNumber("Y Diff to End Pose", yDiff);
        SmartDashboard.putNumber("Degrees Diff to End Pose", rotDifDeg);

        return Math.abs(xDiff) <= tolerance.getX()
                && Math.abs(yDiff) <= tolerance.getY()
                && Math.abs(rotDifDeg) <= tolerance.getRotation().getDegrees();
    }

    @Override
    public void end(boolean interrupted) {
        controller.setEnabled(false);
        chassis.driveFromFieldOrientedChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        SmartDashboard.putString("Status", "Finished");
    }
}
