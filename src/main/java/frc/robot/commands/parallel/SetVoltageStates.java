// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.parallel;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.superstructure.SetLEDState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.subsystems.LED.LEDState;
import frc.robot.subsystems.intake.IntakeFlywheel.IntakeFlywheelState;
import frc.robot.subsystems.intake.IntakeHold.IntakeHoldState;
import frc.robot.subsystems.launcher.LauncherHold.LauncherHoldState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetVoltageStates extends ParallelCommandGroup {
  /** Creates a new SetVoltageStates. */
  public SetVoltageStates(IntakeFlywheelState intakeFlywheelState, IntakeHoldState intakeHoldState, LauncherHoldState launcherHoldState, LEDState ledState) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetVoltageSubsystemState(RobotContainer.m_intakeFlywheel, intakeFlywheelState),
      new SetVoltageSubsystemState(RobotContainer.m_intakeHold, intakeHoldState),
      new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherHoldState),
      new SetLEDState(ledState)
    );
  }

  public SetVoltageStates(IntakeFlywheelState intakeFlywheelState, IntakeHoldState intakeHoldState, LauncherHoldState launcherHoldState) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetVoltageSubsystemState(RobotContainer.m_intakeFlywheel, intakeFlywheelState),
      new SetVoltageSubsystemState(RobotContainer.m_intakeHold, intakeHoldState),
      new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherHoldState)
    );
  }
}
