package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HolonomicChassisSim;

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
    public void execute() {
        chassisSim.driveFromFieldOrientedChassisSpeeds(new ChassisSpeeds(
            xInput.getAsDouble() * Constants.Simulation.MAX_AXIS_SPEED,
            yInput.getAsDouble() * Constants.Simulation.MAX_AXIS_SPEED,
            rotInput.getAsDouble() * Constants.Simulation.MAX_ANGULAR_SPEED
        ));
    }
}
