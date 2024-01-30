// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SetRollerSpeed extends InstantCommand {
  /** Creates a new IntakeNote. */
  private Intake m_intake;
  private double speed;

  public SetRollerSpeed(Intake intake, double newSpeed) {
    m_intake = intake;
    speed = newSpeed;
    
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.driveRoller(speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
