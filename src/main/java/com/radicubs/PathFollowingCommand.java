package com.radicubs;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PathFollowingCommand extends CommandBase {
    protected final HolonomicPathFollower chassis;
    protected HolonomicDriveController controller;
    protected final Timer timer;
    protected final Pose2d endPose, tolerance;
    protected final TrajectoryConstants constants;

    private static HolonomicDriveController getController(TrajectoryConstants constants) {
        return new HolonomicDriveController(
                new PIDController(constants.xControllerkP(), constants.xControllerkI(), constants.xControllerkD()),
                new PIDController(constants.yControllerkP(), constants.yControllerkI(), constants.yControllerkD()),
                new ProfiledPIDController(constants.thetaControllerkP(), constants.thetaControllerkI(),
                        constants.thetaControllerkD(), new TrapezoidProfile.Constraints(
                        constants.maxPathAngularSpeed(), constants.maxPathAngularAcceleration())));
    }

    private static Pose2d getTolerance(TrajectoryConstants constants) {
        return new Pose2d(constants.xTolerance(), constants.yTolerance(),
                Rotation2d.fromRadians(constants.rotTolerance()));
    }

    public <T extends SubsystemBase & HolonomicPathFollower>
    PathFollowingCommand(T chassis, Pose2d endPose, final TrajectoryConstants constants) {
        this.constants = constants;
        this.chassis = chassis;
        this.endPose = endPose;
        this.controller = getController(constants);
        this.tolerance = getTolerance(constants);

        timer = new Timer();

        addRequirements(chassis);
    }

    @Override
    public boolean isFinished() {
        Transform2d diff = chassis.getRobotPose().minus(endPose);
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
        chassis.driveFromFieldOrientedChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        SmartDashboard.putString("Status", "Finished");
    }
}
