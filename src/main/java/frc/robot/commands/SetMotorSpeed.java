// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.MotorFEAR;

public class SetMotorSpeed extends InstantCommand {
  MotorFEAR m_falcon;
  double speed;

  public SetMotorSpeed(MotorFEAR falcon, double velocity) {
    m_falcon = falcon;
    speed = velocity;
    addRequirements(m_falcon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_falcon.set(speed);
  }
}
