package frc.robot.commands;

import com.fasterxml.jackson.databind.deser.std.StringArrayDeserializer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.WaypointFollower;
import frc.robot.subsystems.HolonomicChassisSim;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public class StraightFollower extends SequentialCommandGroup {
    private final HolonomicChassisSim chassisSim;

    public StraightFollower(HolonomicChassisSim chassisSim, Rotation2d startRot, Translation2d... points) {
        this.chassisSim = chassisSim;

        for(int i = 0; i < points.length - 1; i++) {
            Translation2d initial = points[i];
            Translation2d finalPos = points[i + 1];

            int deg = 90;
            if(i%2==1){
                deg=0;
            }

            addCommands(
                    new WaypointFollower(chassisSim,
                            new Rotation2d(Units.degreesToRadians(deg)),
                            new Rotation2d(Units.degreesToRadians(deg)),
                            initial,
                             finalPos));
        }
    }
}