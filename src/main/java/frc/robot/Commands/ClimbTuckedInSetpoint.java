// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb;

public class ClimbManualControl extends Command {
  /** Creates a new ClimbManualControl. */
  Climb m_Climb;

  public ClimbManualControl(Climb newClimb) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Climb = newClimb;
    
    addRequirements(m_Climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Climb.setIntendedPosition(0 + m_Climb.getClimbDeadzone);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
