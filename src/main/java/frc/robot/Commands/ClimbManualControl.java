// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Climb;

public class ClimbManualControl extends Command {
  Climb m_Climb;
  CommandXboxController m_copilotController; 

  public ClimbManualControl(Climb newClimb, CommandXboxController newCopilotController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Climb = newClimb;
    m_copilotController = newCopilotController; 

    addRequirements(m_Climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Climb.climbManualControl(-m_copilotController.getLeftY());
    // up on the stick gives a negative value so we flip its sign

    m_Climb.driveClimb();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Climb.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
