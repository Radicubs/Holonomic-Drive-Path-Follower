// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.StraightFollower;
import frc.robot.commands.TeleOpControl;
import frc.robot.commands.WaypointFollower;
import frc.robot.subsystems.HolonomicChassisSim;

import java.io.IOException;

import static java.lang.Boolean.TRUE;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Joystick joystick;
    private final HolonomicChassisSim chassisSim;

    public SendableChooser<Boolean> pathGeneration;
    public SendableChooser<Boolean> pathWeaver;

    public RobotContainer() {
        joystick = new Joystick(0);
        chassisSim = new HolonomicChassisSim();
        SendableChooser<Boolean> isFieldOriented = new SendableChooser<>();
        isFieldOriented.setDefaultOption("Field Oriented", true);
        isFieldOriented.addOption("Robot Oriented", false);
        SmartDashboard.putData("Control Mode", isFieldOriented);

        SendableChooser<Boolean> pathGeneration = new SendableChooser<>();
        pathGeneration.setDefaultOption("Pathweaver", true);
        pathGeneration.addOption("Point Generation:Spline", false);
        pathGeneration.addOption("Point Generation:Straight", false);
        SmartDashboard.putData("Path Generation", pathGeneration);
        this.pathGeneration = pathGeneration;

        SendableChooser<Boolean> pathWeaver = new SendableChooser<>();
        pathWeaver.setDefaultOption("Get Block", true);
        pathWeaver.addOption("Charge Station", false);
        SmartDashboard.putData("Path Selection", pathWeaver);
        this.pathWeaver = pathWeaver;


        chassisSim.setDefaultCommand(new TeleOpControl(
                chassisSim,
                () -> joystick.getRawAxis(0),
                () -> -joystick.getRawAxis(1),
                () -> joystick.getRawAxis(2),
                isFieldOriented::getSelected
        ));
        configureBindings();
    }


    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {

    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return new StraightFollower(chassisSim, Rotation2d.fromRadians(0), new Translation2d(1, 1), new Translation2d(1, 2), new Translation2d(3, 2), new Translation2d(3, 4));
    }

}
