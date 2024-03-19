// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.lib.utilities.CtreUtils;
import frc.lib.utilities.RevUtils;
import frc.lib.utilities.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase{
  private final int POS_SLOT = 0;
  private final int VEL_SLOT = 0;

  private CANSparkFlex m_driveMotor;
  private CANSparkMax m_turningMotor;
  private CANcoder m_angleEncoder;

  public final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnEncoder;

  private double m_simDriveEncoderPosition;
  private double m_simDriveEncoderVelocity;

  private double m_angleOffset;

  private final SparkPIDController m_driveController;
  private SparkPIDController m_turnController;

  double m_currentAngle;
  double m_lastAngle;
  double m_lastPercentOutput;

  private int m_moduleNumber;

  /**
   * Constructs a SwerveModule.
   *
   * @param moduleNumber The module number
   * @param swerveModuleConstants Swerve modules constants to setup swerve module
   */
  public SwerveModule(int moduleNumber, SwerveModuleConstants swerveModuleConstants) {
    m_moduleNumber = moduleNumber;

    m_driveMotor =
        new CANSparkFlex(
            swerveModuleConstants.driveMotorChannel, MotorType.kBrushless);
    m_turningMotor =
        new CANSparkMax(
            swerveModuleConstants.turningMotorChannel, MotorType.kBrushless);

    m_angleEncoder = new CANcoder(swerveModuleConstants.cancoderID, "CANivore");
    m_angleEncoder.optimizeBusUtilization();
    m_angleEncoder.getAbsolutePosition().setUpdateFrequency(100);
    m_angleOffset = swerveModuleConstants.angleOffset;

    boolean redoSet = true;
    REVLibError error;
    int tries = 0;
    // m_driveMotor.restoreFactoryDefaults();
    while(redoSet){
      error = m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      m_driveMotor.setInverted(true); // MK4i drive motor is inverted
      if(m_driveMotor.getInverted() == true){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    // m_turningMotor.restoreFactoryDefaults();
    while(redoSet){
      error = m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    tries = 0;
    redoSet = true;

    while(redoSet){
      error = m_turningMotor.setSmartCurrentLimit(25);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;
    
    while(redoSet){
      error = m_turningMotor.enableVoltageCompensation(12.6);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = m_turningMotor.disableVoltageCompensation();
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;
    
    while(redoSet){
      m_turningMotor.setInverted(true); // MK4i Steer Motor is inverted
      if(m_turningMotor.getInverted() == true){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    m_angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
    m_angleEncoder.getConfigurator().apply(CtreUtils.generateCanCoderConfig());

    m_driveEncoder = m_driveMotor.getEncoder();

    while(redoSet){
      error = m_driveEncoder.setPositionConversionFactor(DriveConstants.kDriveRevToMeters);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;
    
    while(redoSet){
      error = m_driveEncoder.setVelocityConversionFactor(DriveConstants.kDriveRpmToMetersPerSecond);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;
    
    while(redoSet){
      error = m_driveEncoder.setPosition(0);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;
    
    m_turnEncoder = m_turningMotor.getEncoder();

    while(redoSet){
      error = m_turnEncoder.setPositionConversionFactor(DriveConstants.kTurnRotationsToDegrees);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = m_turnEncoder.setVelocityConversionFactor(DriveConstants.kTurnRotationsToDegrees / 60);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    m_driveController = m_driveMotor.getPIDController();
    m_turnController = m_turningMotor.getPIDController();

    RevUtils.setDriveMotorConfig(m_driveMotor);
    RevUtils.setTurnMotorConfig(m_turningMotor);
  }

  @Override
  public void periodic(){
    if (Constants.kInfoMode) {
      SmartDashboard.putNumber("Swerve/" + "distance" + m_moduleNumber, getPosition().distanceMeters);
      SmartDashboard.putNumber("Swerve/" + "angle " + m_moduleNumber, getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Swerve/" + "Applied Output " + m_moduleNumber, m_driveMotor.getAppliedOutput());
    }
    
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMetersPerSecond(), getHeadingRotation2d());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // if (DriverStation.isTeleop() && DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      // return new SwerveModulePosition(-getDriveMeters(), getHeadingRotation2d());
    // } else {
      return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());
    // }
    
  }

  public void resetAngleToAbsolute() {
    double angle = m_angleEncoder.getAbsolutePosition().getValue() - m_angleOffset;
    m_turnEncoder.setPosition(angle * 360);
  }

  public double getHeadingDegrees() {
    if (Constants.kCurrentMode == Mode.REAL) return m_turnEncoder.getPosition();
    else return m_currentAngle;
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getDriveMeters() {
    if (Constants.kCurrentMode == Mode.REAL) return m_driveEncoder.getPosition();
    else return m_simDriveEncoderPosition;
  }

  public double getDriveMetersPerSecond() {
    if (Constants.kCurrentMode == Mode.REAL) return m_driveEncoder.getVelocity();
    else return m_simDriveEncoderVelocity;
  }

  public void burnFlash() {
    try {
      Thread.sleep(200);
    } catch (Exception e) {}

    System.out.println(m_driveMotor.burnFlash());

    try {
      Thread.sleep(200);
    } catch (Exception e) {}

    System.out.println(m_turningMotor.burnFlash());
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    SwerveModuleState m_desiredState = RevUtils.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput = m_desiredState.speedMetersPerSecond / DriveConstants.kMaxMetersPerSecond;
      if (percentOutput != m_lastPercentOutput) {
        SmartDashboard.putNumber("Swerve/" + "Desired Output " + m_moduleNumber, percentOutput);
        m_driveMotor.set(percentOutput);
        SmartDashboard.putBoolean("Swerve/ " + "Is Chattering", Math.abs(percentOutput) > Math.abs(m_driveMotor.getAppliedOutput()));
        m_lastPercentOutput = percentOutput;
      }
    } else {
      int DRIVE_PID_SLOT = VEL_SLOT;
      m_driveController.setReference(
          m_desiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, DRIVE_PID_SLOT);
    }

    double angle = m_desiredState.angle.getDegrees();

    

    if ((m_desiredState.angle.getDegrees() != m_lastAngle && 
      Math.abs(m_desiredState.speedMetersPerSecond)
        >= (DriveConstants.kMaxMetersPerSecond * DriveConstants.kSteerVelocityDeadzone))
        || RobotContainer.m_drivebase.getXWheels()) {
          m_turnController.setReference(angle, CANSparkMax.ControlType.kPosition, POS_SLOT);
          m_lastAngle = angle;
    }
    
    

    if (Constants.kCurrentMode == Mode.SIM) {
      simUpdateDrivePosition(m_desiredState);
      // simTurnPosition(angle);
      m_currentAngle = angle;
    }
  }

  private void simUpdateDrivePosition(SwerveModuleState state) {
    m_simDriveEncoderVelocity = state.speedMetersPerSecond;
    double distancePer20Ms = m_simDriveEncoderVelocity / 50.0;

    m_simDriveEncoderPosition += distancePer20Ms;
  }
}
