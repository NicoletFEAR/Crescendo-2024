// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.templates.MultiMotorPositionSubsystem;
import frc.robot.subsystems.templates.SuperstructureSubsystem;
import frc.robot.subsystems.templates.MultiMotorPositionSubsystem.MultiMotorPositionSubsystemState;
import frc.robot.subsystems.templates.SuperstructureSubsystem.SuperstructureState;

public class SetMultiMotorPositionSubsystemState extends Command {
  /** Creates a new SetMechState. */
  private MultiMotorPositionSubsystem m_subsystem;
  private MultiMotorPositionSubsystemState m_subsystemState;

  private SuperstructureSubsystem m_superstructure;
  private SuperstructureState m_superstructureState;

  public SetMultiMotorPositionSubsystemState(MultiMotorPositionSubsystem subsystem, MultiMotorPositionSubsystemState subsystemState, SuperstructureSubsystem superstructure, SuperstructureState superstructureState) {
    m_subsystem = subsystem;
    m_subsystemState = subsystemState;
    m_superstructure = superstructure;
    m_superstructureState = superstructureState;

    addRequirements(m_subsystem);
  }

  public SetMultiMotorPositionSubsystemState(MultiMotorPositionSubsystem subsystem, MultiMotorPositionSubsystemState subsystemState) {
    m_subsystem = subsystem;
    m_subsystemState = subsystemState;
    m_superstructure = null;
    m_superstructureState = null;

    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setDesiredState(m_subsystemState, true);
    if (m_superstructure != null || m_superstructureState != null) {
      m_superstructure.setDesiredState(m_superstructureState);
      m_superstructure.setCurrentState(m_superstructure.getTransitionState());
    }
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
