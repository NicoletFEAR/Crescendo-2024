// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.IntakeNote;
import frc.robot.subsystems.Intake;

//----------------------------------------------//

public class RobotContainer {
  public static CommandXboxController m_driverController = new CommandXboxController(0);

  private final Intake m_intakeSubsystem = new Intake();

  public RobotContainer() {
    m_intakeSubsystem.setDefaultCommand(new IntakeNote(m_intakeSubsystem, m_driverController.getRightY()));
    configureBindings();
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
