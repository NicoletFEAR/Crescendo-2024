// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.superstructure.SetVelocitySubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.commands.waits.WaitForLaunchNote;
import frc.robot.commands.waits.WaitForNoLaunchNote;
import frc.robot.subsystems.launcher.LauncherFlywheel.LauncherFlywheelState;
import frc.robot.subsystems.launcher.LauncherHold.LauncherHoldState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PositionNoteInLauncher extends SequentialCommandGroup {
  /** Creates a new PositionNoteInLauncher. */
  public PositionNoteInLauncher() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Brings note out until beam break isnt tripped
      new ParallelCommandGroup(new SetVoltageSubsystemState(RobotContainer.m_launcherHold, LauncherHoldState.ADJUST_NOTE_OUT),
        new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, LauncherFlywheelState.ADJUST_NOTE_OUT))
      .raceWith(new WaitForNoLaunchNote()),

      // Brings note in until beam break is tripped
      new ParallelCommandGroup(new SetVoltageSubsystemState(RobotContainer.m_launcherHold, LauncherHoldState.ADJUST_NOTE_IN),
        new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, LauncherFlywheelState.ADJUST_NOTE_IN))
      .raceWith(new WaitForLaunchNote()),

      // Brings it in for a set period of time
      new ParallelCommandGroup(new SetVoltageSubsystemState(RobotContainer.m_launcherHold, LauncherHoldState.ADJUST_NOTE_IN),
        new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, LauncherFlywheelState.ADJUST_NOTE_IN)),

      // Wait command for how long to bring it back in for
      // You have to tune this!!  
      new WaitCommand(.05),

      // Stops motors
      new ParallelCommandGroup(new SetVoltageSubsystemState(RobotContainer.m_launcherHold, LauncherHoldState.OFF),
        new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, LauncherFlywheelState.OFF))
    );
  }
}
