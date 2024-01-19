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

import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetWristGoal;
import frc.robot.subsystems.Intake;

//----------------------------------------------//

public class RobotContainer {
  public static CommandXboxController m_driverController = new CommandXboxController(0);

  private final Intake m_intakeSubsystem = new Intake();

  public float multiplier = 1;

  public RobotContainer() {
    //m_swerveSubsystem.setDefaultCommand(new Drive(m_swerveSubsystem, m_driverController.getRightY() * multiplier));
    m_intakeSubsystem.setDefaultCommand(new IntakeCommand(m_intakeSubsystem));
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.x().onTrue(new SetWristGoal(m_intakeSubsystem, 1));
    m_driverController.a().onTrue(new SetWristGoal(m_intakeSubsystem, 2));
    m_driverController.b().onTrue(new SetWristGoal(m_intakeSubsystem, 3));
  }

  public Command getAutonomousCommand() {
    // TODO Auto-generated method stub

    return null;
  }
}
