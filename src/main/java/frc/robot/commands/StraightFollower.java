package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.HolonomicChassisSim;

public class StraightFollower extends SequentialCommandGroup {
    private  HolonomicChassisSim chassis;
    public StraightFollower(HolonomicChassisSim chassis, Pose2d ...waypoints) {
        for(int i = 0;i < waypoints.length;i++){
            addCommands(
                    new MoveToPose(chassis, waypoints[i])
            );


        }
    }
}
