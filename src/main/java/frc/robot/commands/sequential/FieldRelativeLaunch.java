// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.utilities.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.commands.drivebase.KeepResettingPoseWithVision;
import frc.robot.commands.drivebase.ResetPoseWithVision;
import frc.robot.commands.drivebase.TurnToAngle;
import frc.robot.subsystems.launcher.LauncherSuperstructure.LauncherSuperstructureState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FieldRelativeLaunch extends SequentialCommandGroup {
  /** Creates a new FieldRelativeLaunch. */
  public FieldRelativeLaunch() {
    addCommands(
      new ResetPoseWithVision(),

      new ParallelRaceGroup(
        new KeepResettingPoseWithVision(),

        new SequentialCommandGroup(
            // Turns then X wheels when done turning, and while doing that rev up wheels
            new SequentialCommandGroup(new TurnToAngle(RobotContainer.m_drivebase),
              new InstantCommand(RobotContainer.m_drivebase::toggleXWheels))
            .alongWith(RobotContainer.m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.FIELD_BASED_PREP)),

            // Launch then wait till done launching then go back to zero
            RobotContainer.m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.FIELD_BASED_LAUNCH),
            new WaitCommand(.1),
            RobotContainer.m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOWED)
        )
      )
    );
  }
}
