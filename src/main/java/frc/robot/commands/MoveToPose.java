package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.SplineParameterizer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.HolonomicPathFollower;
import frc.robot.subsystems.SimulationChassis;

import java.sql.SQLOutput;
import java.util.List;

import static java.lang.Math.atan;

public class MoveToPose extends CommandBase {

    private HolonomicDriveController controller;
    private HolonomicPathFollower chassis;
    private Pose2d endPose;
    private Trajectory trajectory;

    private Pose2d tolerance;
    private Timer timer;
    private double xDiff;
    private double yDiff;
    private double hypoAngle;

    private Pose2d startPose;

    public <T extends SubsystemBase & HolonomicPathFollower> MoveToPose(T chassis, Pose2d endPose, HolonomicDriveController controller, Pose2d tolerance) {
        this.chassis = chassis;
        this.endPose = endPose;
        this.controller = controller;
        this.tolerance = tolerance;

        timer = new Timer();
    }

    @Override
    public void initialize() {
        xDiff = endPose.getX() - chassis.getRobotPose().getX();
        yDiff = endPose.getY() - chassis.getRobotPose().getY();
        hypoAngle = Units.radiansToDegrees(atan(yDiff/xDiff));
        SmartDashboard.putString("Status", "Running");
        timer.start();
        try {
            trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(chassis.getRobotPose().getTranslation(), Rotation2d.fromDegrees(hypoAngle)),
                    List.of(),
                    new Pose2d(endPose.getTranslation(), Rotation2d.fromDegrees(hypoAngle)),
                    new TrajectoryConfig(Constants.TrajectoryFollower.MAX_PATH_SPEED, Constants.TrajectoryFollower.MAX_PATH_ACCELERATION));
        }catch (SplineParameterizer.MalformedSplineException e){
            System.out.println("Unable to generate straight trajectory to point");
            System.out.println("This may be because the robot is already at the endpoint");
            System.out.println(e);
            this.cancel();
        }
        chassis.displayTrajectory(trajectory);
    }

    @Override
    public void execute() {
        Trajectory.State state = trajectory.sample(timer.get());
        ChassisSpeeds speeds = controller.calculate(chassis.getRobotPose(), trajectory.sample(timer.get()), endPose.getRotation());
        chassis.driveFromRobotOrientedChassisSpeeds(speeds);
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

