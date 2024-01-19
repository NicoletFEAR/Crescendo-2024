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
  public static CommandXboxController m_copilotController = new CommandXboxController(0);

  private final Intake m_intakeSubsystem = new Intake();

  public RobotContainer() {
    m_intakeSubsystem.setDefaultCommand(new IntakeCommand(m_intakeSubsystem, m_copilotController));
    configureBindings();
  }

  private void configureBindings() {
    m_copilotController.x().onTrue(new SetWristGoalToAmp(m_intakeSubsystem));
    m_copilotController.a().onTrue(new SetWristGoalToFloor(m_intakeSubsystem));
    m_copilotController.b().onTrue(new SetWristGoalToTrap(m_intakeSubsystem));

    // add intake and outtake to triggers
    // add move to launch to bumpers
  }

  public Command getAutonomousCommand() {
    // TODO Auto-generated method stub

    return null;
  }
}
