// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.lib.utilities.CtreUtils;
import frc.lib.utilities.RevUtils;
import frc.lib.utilities.SwerveModuleConstants;
import java.util.Map;

public class SwerveModule extends SubsystemBase {
  private final int POS_SLOT = 0;
  private final int VEL_SLOT = 0;

  private CANSparkFlex m_driveMotor;
  private CANSparkFlex m_turningMotor;
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

  private SimpleWidget m_moduleAngleWidget;
  private SimpleWidget m_moduleSpeedWidget;

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
        new CANSparkFlex(
            swerveModuleConstants.turningMotorChannel, MotorType.kBrushless);

    m_angleEncoder = new CANcoder(swerveModuleConstants.cancoderID, "rio");
    m_angleOffset = swerveModuleConstants.angleOffset;

    m_driveMotor.restoreFactoryDefaults();
    RevUtils.setDriveMotorConfig(m_driveMotor);
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_driveMotor.setInverted(true); // MK4i drive motor is inverted

    m_turningMotor.restoreFactoryDefaults();
    RevUtils.setTurnMotorConfig(m_turningMotor);
    m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_turningMotor.setSmartCurrentLimit(25);
    m_turningMotor.enableVoltageCompensation(12.6);
    m_turningMotor.setInverted(true); // MK4i Steer Motor is inverted

    m_angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
    m_angleEncoder.getConfigurator().apply(CtreUtils.generateCanCoderConfig());

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(DriveConstants.kDriveRevToMeters);
    m_driveEncoder.setVelocityConversionFactor(DriveConstants.kDriveRpmToMetersPerSecond);
    m_driveEncoder.setPosition(0);

    m_turnEncoder = m_turningMotor.getEncoder();
    m_turnEncoder.setPositionConversionFactor(DriveConstants.kTurnRotationsToDegrees);
    m_turnEncoder.setVelocityConversionFactor(DriveConstants.kTurnRotationsToDegrees / 60);

    m_driveController = m_driveMotor.getPIDController();
    m_turnController = m_turningMotor.getPIDController();
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
    return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());
  }

  public void resetAngleToAbsolute() {
    double angle = m_angleEncoder.getAbsolutePosition().getValue() - m_angleOffset;
    m_turnEncoder.setPosition(angle * 360);
  }

  public double getHeadingDegrees() {
    if (RobotBase.isReal()) return m_turnEncoder.getPosition();
    else return m_currentAngle;
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getDriveMeters() {
    if (RobotBase.isReal()) return m_driveEncoder.getPosition();
    else return m_simDriveEncoderPosition;
  }

  public double getDriveMetersPerSecond() {
    if (RobotBase.isReal()) return m_driveEncoder.getVelocity();
    else return m_simDriveEncoderVelocity;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = RevUtils.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.kMaxMetersPerSecond;
      m_driveMotor.set(percentOutput);
    } else {
      int DRIVE_PID_SLOT = VEL_SLOT;
      m_driveController.setReference(
          desiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, DRIVE_PID_SLOT);
    }

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond)
                <= (DriveConstants.kMaxMetersPerSecond
                    * 0.01)) // Prevent rotating module if speed is less than 1%. Prevents
            // Jittering.
            ? m_lastAngle
            : desiredState.angle.getDegrees();
    angle = desiredState.angle.getDegrees();
    m_turnController.setReference(angle, CANSparkMax.ControlType.kPosition, POS_SLOT);
    m_lastAngle = angle;

    if (RobotBase.isSimulation()) {
      simUpdateDrivePosition(desiredState);
      // simTurnPosition(angle);
      m_currentAngle = angle;
    }
  }

  @Override
  public void periodic() {}

  private void simUpdateDrivePosition(SwerveModuleState state) {
    m_simDriveEncoderVelocity = state.speedMetersPerSecond;
    double distancePer20Ms = m_simDriveEncoderVelocity / 50.0;

    m_simDriveEncoderPosition += distancePer20Ms;
  }

  public void tuningInit() {}

  public void tuningPeriodic() {
    m_driveController.setP(SwerveDrive.drivekp.get());
    m_driveController.setI(SwerveDrive.driveki.get());
    m_driveController.setD(SwerveDrive.drivekd.get());
    m_driveController.setFF(SwerveDrive.drivekff.get());
    m_driveMotor.setOpenLoopRampRate(SwerveDrive.driveRampRate.get());

    m_turnController.setP(SwerveDrive.drivekp.get());
    m_turnController.setI(SwerveDrive.driveki.get());
    m_turnController.setD(SwerveDrive.drivekd.get());
    m_turnController.setFF(SwerveDrive.drivekff.get());
  }

  public void infoInit() {
    m_moduleAngleWidget =
        RobotContainer.infoTab
            .add("Module Angle " + m_moduleNumber, getHeadingDegrees())
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(5 + m_moduleNumber * 2, 2);

    m_moduleSpeedWidget =
        RobotContainer.infoTab
            .add("Module Speed " + m_moduleNumber, Units.metersToFeet(getDriveMetersPerSecond()))
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -16, "max", 16))
            .withPosition(5 + m_moduleNumber * 2, 4)
            .withSize(2, 2);
  }

  public void infoPeriodic() {
    m_moduleAngleWidget.getEntry().setDouble(getHeadingDegrees());
    m_moduleSpeedWidget.getEntry().setDouble(Units.metersToFeet(getDriveMetersPerSecond()));
  }
}
