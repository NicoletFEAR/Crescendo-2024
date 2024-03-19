// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.templates.MultiMotorPositionSubsystem;
import frc.lib.templates.MultiMotorPositionSubsystem.MultiMotorPositionSubsystemState;

public class SetMultiMotorPositionSubsystemState extends Command {
  /** Creates a new SetMechState. */
  private MultiMotorPositionSubsystem m_subsystem;
  private MultiMotorPositionSubsystemState m_subsystemState;

  public SetMultiMotorPositionSubsystemState(MultiMotorPositionSubsystem subsystem, MultiMotorPositionSubsystemState subsystemState) {
    m_subsystem = subsystem;
    m_subsystemState = subsystemState;

    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setDesiredState(m_subsystemState, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.atSetpoint();
  }
}
