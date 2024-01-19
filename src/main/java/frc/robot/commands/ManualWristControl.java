// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class IntakeCommand extends Command {
  private Intake m_intake;
  private CommandXboxController m_copilotController;

  /** Creates a new Intake. */
  public IntakeCommand(Intake intake, CommandXboxController newCopilotController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_copilotController = newCopilotController;

    addRequirements(intake, m_copilotController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.manualWristControl(m_copilotController.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
