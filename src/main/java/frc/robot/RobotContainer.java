// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.Drive;
import frc.robot.subsystems.Swerve;

//----------------------------------------------//

public class RobotContainer {
  public static CommandXboxController m_driverController = new CommandXboxController(0);

  private final Swerve m_swerveSubsystem = new Swerve();
  public Drive drive = new Drive(m_swerveSubsystem, 0.2);

  public float multiplier = 1;

  public RobotContainer() {
    //m_swerveSubsystem.setDefaultCommand(new Drive(m_swerveSubsystem, m_driverController.getRightY() * multiplier));
    m_swerveSubsystem.setDefaultCommand(new Drive(m_swerveSubsystem, 0.7));
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.x().whileTrue(
      new Drive(m_swerveSubsystem, 1));
      
  }

  public Command getAutonomousCommand() {
    return drive;
  }
}
