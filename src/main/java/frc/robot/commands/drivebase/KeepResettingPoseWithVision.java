// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utilities.LimelightHelpers;
import frc.lib.utilities.LimelightHelpers.PoseEstimate;
import frc.robot.RobotContainer;

public class KeepResettingPoseWithVision extends Command {
  /** Creates a new KeepResettingPoseWithVision. */
  PoseEstimate m_estimate;
  public KeepResettingPoseWithVision() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-launch");
    // if (m_estimate.isPoseTrustworthy()) {
      RobotContainer.m_drivebase.addVisionEstimate(m_estimate.pose, m_estimate.timestampSeconds);
    // }
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
