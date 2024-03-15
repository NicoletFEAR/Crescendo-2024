// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.utilities.LimelightHelpers;
import frc.lib.utilities.LimelightHelpers.PoseEstimate;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetPoseWithVision extends InstantCommand {
  private PoseEstimate estimate;

  public ResetPoseWithVision() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-launch");
    // if (estimate.isPoseTrustworthy()) {
      RobotContainer.m_drivebase.updateEstimatorWithPose(estimate.pose);
    // }
    
    // if(estimate.isPoseTrustworthy()){
    //   System.out.println("Pose is trustworthy");
    // }
    // else{
    //   System.out.println("Pose is not trustworthy");
    // }
  }
}
