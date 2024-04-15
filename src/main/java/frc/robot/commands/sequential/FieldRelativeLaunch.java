// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
// import frc.robot.commands.drivebase.ResetPoseWithVision;
import frc.robot.commands.drivebase.TurnToAngle;
import frc.robot.commands.drivebase.TurnToAngle.AngleToTurn;
import frc.robot.commands.superstructure.SetLEDState;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVelocitySubsystemState;
import frc.robot.subsystems.LED.LEDState;
import frc.robot.subsystems.launcher.LauncherSuperstructure.LauncherSuperstructureState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FieldRelativeLaunch extends SequentialCommandGroup {
  /** Creates a new FieldRelativeLaunch. */
  public FieldRelativeLaunch(LauncherSuperstructureState desiredState, AngleToTurn angleToTurn) {
    addCommands(
      // new ResetPoseWithVision(),


      new SequentialCommandGroup(
          // Turns then X wheels when done turning, and while doing that rev up wheels
          new SequentialCommandGroup(new TurnToAngle(RobotContainer.m_drivebase, angleToTurn),
            new InstantCommand(RobotContainer.m_drivebase::toggleXWheels))
          .alongWith(new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, desiredState.launcherFlywheelState),
                      new SetPositionSubsystemState(RobotContainer.m_launcherWrist, desiredState.launcherWristState),
                      new SetLEDState(LEDState.GREEN_LAUNCHER_LEDS)
                    ),

          // Launch then wait till done launching then go back to zero
          RobotContainer.m_launcherSuperstructure.setSuperstructureState(desiredState),
          new WaitCommand(.1),
          RobotContainer.m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOWED)
            .alongWith(
              new SetLEDState(LEDState.GREEN_FLASHING, 1.0, LEDState.TEAL_STOW)
            )
      )
    );
  }
}
