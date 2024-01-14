// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class RunLaunchMotors extends Command {

  private double m_leftVoltage;
  private double m_rightVoltage;
  private Launcher m_launcher; 
  /** Creates a new RunLaunchMotors. */
  public RunLaunchMotors(Launcher launcher, double leftVoltage, double rightVoltage) {
    m_leftVoltage = leftVoltage;
    m_rightVoltage = rightVoltage;
    m_launcher = launcher;
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_launcher.runMotor(m_leftVoltage, m_rightVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_launcher.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
