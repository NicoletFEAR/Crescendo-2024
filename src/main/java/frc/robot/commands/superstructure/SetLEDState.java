// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLEDState extends InstantCommand {
  LEDState m_state;
  double m_seconds = -1;
  LEDState m_postTimerState;

  public SetLEDState(LEDState state) {
    m_state = state;
  }

  public SetLEDState(LEDState state, double seconds, LEDState postTimerState) {
    m_state = state;
    m_seconds = seconds;
    m_postTimerState = postTimerState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_seconds == -1) {
      LED.setState(m_state);
    } else {
      LED.setState(m_state, m_seconds, m_postTimerState);
    }
    
  }
}
