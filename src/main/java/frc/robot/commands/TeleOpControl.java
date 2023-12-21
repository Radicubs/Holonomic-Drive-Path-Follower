package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SimulationChassis;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleOpControl extends CommandBase {
    SimulationChassis chassisSim;
    DoubleSupplier xInput;
    DoubleSupplier yInput;
    DoubleSupplier rotInput;
    BooleanSupplier isFieldOriented;
    public TeleOpControl(SimulationChassis chassisSim, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rotInput, BooleanSupplier isFieldOriented){
        this.chassisSim = chassisSim;
        this.xInput = xInput;
        this.yInput = yInput;
        this.rotInput = rotInput;
        this.isFieldOriented = isFieldOriented;
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
        SmartDashboard.putNumber("Robot Pose", chassisSim.getRobotPose().getX());
        ChassisSpeeds speeds = new ChassisSpeeds(
                xInput.getAsDouble() * 3,
                yInput.getAsDouble() * 3,
                rotInput.getAsDouble() * 3
        );
        if(isFieldOriented.getAsBoolean()){
            chassisSim.driveFromFieldOrientedChassisSpeeds(speeds);
        }
        else{
            chassisSim.driveFromRobotOrientedChassisSpeeds(speeds);
        }
    }
}
