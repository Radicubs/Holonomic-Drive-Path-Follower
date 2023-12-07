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
import frc.robot.Constants;
import frc.robot.subsystems.HolonomicChassisSim;

import java.util.List;

import static java.lang.Math.atan;

public class MoveToPose extends CommandBase {

    private  HolonomicDriveController controller;
    private  HolonomicChassisSim chassis;
    private Pose2d endPose;
    private Trajectory trajectory;

    private Pose2d tolerance;
    private Timer timer;
    private double xDiff;
    private double yDiff;
    private double hypoAngle;

    private Pose2d startPose;

    public MoveToPose(HolonomicChassisSim chassis, Pose2d endPose) {
        timer = new Timer();
        addRequirements(chassis);
        this.tolerance = new Pose2d(new Translation2d(.03,.03), Rotation2d.fromDegrees(2));
        this.chassis = chassis;
        this.endPose = endPose;
        controller = new HolonomicDriveController(
                new PIDController(2,0 ,0),
                new PIDController(2,0, 0),
                new ProfiledPIDController(2, 0, 0,
                        new TrapezoidProfile.Constraints(Constants.Simulation.MAX_PATH_ANGULAR_SPEED, Constants.Simulation.MAX_PATH_ANGULAR_ACCELERATION)));


    }

    @Override
    public void initialize() {
        xDiff = endPose.getX() - chassis.getRobotPose().getX();
        yDiff = endPose.getY() - chassis.getRobotPose().getY();
        hypoAngle = Units.radiansToDegrees(atan(yDiff/xDiff));
        SmartDashboard.putString("Status","Running");
        timer.start();
        try {
            trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(chassis.getRobotPose().getTranslation(), Rotation2d.fromDegrees(hypoAngle)),
                    List.of(),
                    new Pose2d(endPose.getTranslation(), Rotation2d.fromDegrees(hypoAngle)),
                    new TrajectoryConfig(Constants.Simulation.MAX_PATH_SPEED, Constants.Simulation.MAX_PATH_ACCELERATION));
        }catch (SplineParameterizer.MalformedSplineException e ){
            System.out.println("Already at end position");
        }
        chassis.displayTrajectory(trajectory);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = controller.calculate(chassis.getRobotPose(), trajectory.sample(timer.get()), endPose.getRotation());
        chassis.driveFromRobotOrientedChassisSpeeds(speeds, false);
    }

    @Override
    public boolean isFinished() {
        Transform2d diff = chassis.getRobotPose().minus(endPose);
        double xDiff = diff.getX();
        double yDiff = diff.getY();
        double rotDifDeg = diff.getRotation().getDegrees();

        SmartDashboard.putNumber("Robot Pose", chassis.getRobotPose().getX());
        SmartDashboard.putNumber("Ending Pose", endPose.getX());

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
        SmartDashboard.putString("Status","Finished");
    }
}

