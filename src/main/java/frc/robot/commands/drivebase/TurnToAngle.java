// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.lib.utilities.GeometryUtils;

public class TurnToAngle extends Command {
  /** Creates a new TurnToAngle. */

  private SwerveDrive m_drivebase;
  private double m_targetAngle = 0;
  private double deadBand = 1;

  /**
   *
   *
   * <h3>TurnToAngle</h3>
   *
   * Turns to a specified angle using a pid controller
   *
   * @param m_drivebase The swerve drive that moves the robot
   * @param targetAngle Target angle to turn to
   */
  public TurnToAngle(SwerveDrive drivebase, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivebase = drivebase;
    m_targetAngle = targetAngle;

    addRequirements(m_drivebase);
  }

  /**
   *
   *
   * <h3>TurnToAngle</h3>
   *
   * Turns to the speaker using a pid controller
   *
   * @param m_drivebase The swerve drive that moves the robot
   */
  public TurnToAngle(SwerveDrive drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivebase = drivebase;
    m_targetAngle = 20;

    addRequirements(m_drivebase);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double difference = (m_drivebase.getYawDegrees() - m_targetAngle);
    double speeds;

    if(difference < deadBand){
      difference = 0.0;
      speeds = difference;
    }
    else if(difference  < 30){
      speeds = (difference / 60);
    }
    else{
      speeds = difference * 0.5;
    }

    m_drivebase.drive(0, 0, speeds, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_targetAngle = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_drivebase.getYawDegrees() - m_targetAngle) < deadBand) {
      return true;
    } else {
      return false;
    }
  }
}
