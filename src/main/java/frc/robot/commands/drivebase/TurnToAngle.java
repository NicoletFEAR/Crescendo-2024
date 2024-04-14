// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.lib.utilities.GeometryUtils;

public class TurnToAngle extends Command {
  /** Creates a new TurnToAngle. */
  private PIDController angleController = new PIDController(.016, 0.003, 0.0);

  private SwerveDrive m_drivebase;
  private double m_targetAngle = -1;
  private double deadBand = 2;

  private double kff = 0.0;

  private AutoPos autoPosition = AutoPos.NONE;

  private AngleToTurn m_angleToTurn = AngleToTurn.OTHER;

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
  public TurnToAngle(SwerveDrive drivebase, AngleToTurn angleToTurn) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivebase = drivebase;
    m_angleToTurn = angleToTurn;

    if (angleToTurn == AngleToTurn.SPEAKER) {
      // RobotContainer.mainTab.add("Turn To Angle Controller", angleController);
      
    }

    // SmartDashboard.putNumber("Turn To Angle KFF", kff);
    


    addRequirements(m_drivebase);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (RobotContainer.m_drivebase.getIsSetGyroRequestPresent()) {
      if (RobotContainer.m_drivebase.getIsGyroRequestAmpSide()) {
        autoPosition = AutoPos.AMP;
      } else {
        autoPosition = AutoPos.SOURCE;
      }
    }

    m_drivebase.drive(0, 0, 0, true, true);

    angleController.setIZone(3);

    if (m_angleToTurn == AngleToTurn.SPEAKER) {
      m_targetAngle = m_drivebase.calculateAngleToSpeaker() < 0 ? m_drivebase.calculateAngleToSpeaker() + 180 : m_drivebase.calculateAngleToSpeaker() - 180;

      var alliance = DriverStation.getAlliance();

      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        m_targetAngle += -5;
      } else {
        m_targetAngle -= 185;
      }
      m_targetAngle += autoPosition.getOffset();
      
      /*
       *  this part is here instead of the constructor because the constructor happens on robot init, 
       *    but initialize happens when ari first presses the button
       */
    } else if (m_angleToTurn == AngleToTurn.AMP) {
      m_targetAngle = m_drivebase.calculateAngleToAmp() < 0 ? m_drivebase.calculateAngleToAmp() + 180 : m_drivebase.calculateAngleToAmp() - 180;

      var alliance = DriverStation.getAlliance();

      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        m_targetAngle += 0;
      } else {
        m_targetAngle -= 180;
      }
      m_targetAngle += autoPosition.getOffset();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_angleToTurn == AngleToTurn.SPEAKER) {
      m_targetAngle = m_drivebase.calculateAngleToSpeaker() < 0 ? m_drivebase.calculateAngleToSpeaker() + 180 : m_drivebase.calculateAngleToSpeaker() - 180;

      var alliance = DriverStation.getAlliance();

      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        m_targetAngle += -5;
      } else {
        m_targetAngle -= 185;
      }
      m_targetAngle += autoPosition.getOffset();
    }

    // kff = .02;


    double speeds =
        angleController.calculate(
            GeometryUtils.getAdjustedYawDegrees(m_drivebase.getYawDegrees(), m_targetAngle), 180);

    kff = speeds > 0.0 ? Math.abs(kff) : -Math.abs(kff);

    speeds = MathUtil.clamp(speeds + kff, -1, 1);
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

  public enum AngleToTurn {
    SPEAKER,
    AMP,
    OTHER
  }

  public enum AutoPos {
    AMP(-60),
    SOURCE(60),
    NONE(0);

    private double offset;

    private AutoPos(double offset) {
      this.offset = offset;
    }

    public double getOffset() {
      return offset;
    }
  }
}