// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotStateManager.RobotState;
// import frc.robot.subsystems.launcher.LauncherSuperstructure;
import frc.robot.subsystems.launcher.LauncherSuperstructure.LauncherSuperstructureState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SubwooferCommandScheduler extends InstantCommand {
  public SubwooferCommandScheduler() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.m_launcherSuperstructure.getDesiredState() != LauncherSuperstructureState.SUBWOOFER_PREPARE && RobotContainer.m_launcherSuperstructure.getDesiredState() != LauncherSuperstructureState.SUBWOOFER) {
      RobotContainer.m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.SUBWOOFER_PREPARE).schedule();
    } else if (RobotContainer.m_launcherSuperstructure.getDesiredState() != LauncherSuperstructureState.SUBWOOFER) {
      RobotContainer.m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.SUBWOOFER)
        .andThen(
          RobotContainer.m_robotStateManager.setSuperstructureState(RobotState.TRAVEL)
        ).schedule();
    }
  }
}
