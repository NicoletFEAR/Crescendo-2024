// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.lib.utilities.GeometryUtils;

public class TurnToAngle extends Command {
  /** Creates a new TurnToAngle. */
  private PIDController angleController = new PIDController(.02, 0.025, 0.001);
;

  private SwerveDrive m_drivebase;
  private double m_targetAngle = -1;
  private double deadBand = 1;

  private boolean turnToSpeaker = false;

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
    turnToSpeaker = true;

    addRequirements(m_drivebase);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (turnToSpeaker) {
      m_targetAngle = m_drivebase.calculateAngleToSpeaker() < 0 ? m_drivebase.calculateAngleToSpeaker() + 180 : m_drivebase.calculateAngleToSpeaker() - 180;

      var alliance = DriverStation.getAlliance();

      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        m_targetAngle += 5;
      } else {
        m_targetAngle -= 175;
      }
      
      /*
       *  this part is here instead of the constructor because the constructor happens on robot init, 
       *    but initialize happens when ari first presses the button
       */
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speeds =
        angleController.calculate(
            GeometryUtils.getAdjustedYawDegrees(m_drivebase.getYawDegrees(), m_targetAngle), 180);
    m_drivebase.drive(0, 0, speeds, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(GeometryUtils.getAdjustedYawDegrees(m_drivebase.getYawDegrees(), m_targetAngle) - 180) < deadBand) {
      m_drivebase.drive(0, 0, 0, true, true);
      return true;
    } else {
      return false;
    }
  }
}