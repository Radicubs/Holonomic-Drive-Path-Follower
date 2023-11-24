package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HolonomicChassisSim;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class TeleOpControl extends CommandBase {
    HolonomicChassisSim chassisSim;
    DoubleSupplier xInput;
    DoubleSupplier yInput;
    DoubleSupplier rotInput;
    public TeleOpControl(HolonomicChassisSim chassisSim, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rotInput){
        this.chassisSim = chassisSim;
        this.xInput = xInput;
        this.yInput = yInput;
        this.rotInput = rotInput;
        addRequirements(chassisSim);
    }

    @Override
    public void initialize() {
        /*Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(2, 2), new Rotation2d()),
                List.of(),
                new Pose2d(new Translation2d(6, 3), new Rotation2d()),
                new TrajectoryConfig(Constants.Simulation.MAX_AXIS_SPEED, Constants.Simulation.MAX_ACCELERATION));
        chassisSim.displayTrajectory(trajectory);*/
    }

    @Override
    public void execute() {
        chassisSim.driveFromFieldOrientedChassisSpeeds(new ChassisSpeeds(
            xInput.getAsDouble() * Constants.Simulation.MAX_AXIS_SPEED,
            yInput.getAsDouble() * Constants.Simulation.MAX_AXIS_SPEED,
            rotInput.getAsDouble() * Constants.Simulation.MAX_ANGULAR_SPEED
        ));
    }
}
