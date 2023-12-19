package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HolonomicPathFollower;

public class FollowTrajectory extends CommandBase {

    private HolonomicPathFollower chassis;

    private Trajectory trajectory;
    private HolonomicDriveController controller;
    private Timer timer;
    private Pose2d startingPose, endingPose, tolerance;
    private boolean followTrajectoryHeading;

    public <T extends SubsystemBase & HolonomicPathFollower> FollowTrajectory(T chassis, Trajectory trajectory, HolonomicDriveController controller, Pose2d tolerance, boolean followHeading){
        this.chassis = chassis;
        this.trajectory = trajectory;
        this.controller = controller;
        this.tolerance = tolerance;
        this.followTrajectoryHeading = followHeading;
        timer = new Timer();

        startingPose = trajectory.getInitialPose();
        endingPose = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;
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
        ChassisSpeeds speeds = controller.calculate(chassis.getRobotPose(), state, followTrajectoryHeading ? state.poseMeters.getRotation() : endingPose.getRotation());
        SmartDashboard.putNumber("PID Target X", controller.getXController().getSetpoint());
        SmartDashboard.putNumber("PID Target Y", controller.getYController().getSetpoint());
        SmartDashboard.putNumber("PID Target Angle", controller.getThetaController().getSetpoint().position);
        chassis.driveFromRobotOrientedChassisSpeeds(speeds);
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
        chassis.driveFromRobotOrientedChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        SmartDashboard.putString("Status", "Finished");
    }
}
