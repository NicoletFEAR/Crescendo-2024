// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.templates;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.templates.PositionSubsystem.PositionSubsystemState;
import frc.robot.subsystems.templates.PositionSubsystem.PositionSubsystemType;
import frc.robot.subsystems.templates.SubsystemConstants.TalonFXPositionSubsystemConstants;
import frc.lib.utilities.CtreUtils;
import frc.lib.utilities.LoggedShuffleboardTunableNumber;
import org.littletonrobotics.junction.Logger;

public abstract class TalonFXPositionSubsystem extends SubsystemBase {

  public TalonFXPositionSubsystemConstants m_constants;

  protected final TalonFX m_master;
  protected final TalonFX[] m_slaves;

  protected LoggedShuffleboardTunableNumber m_kp;
  protected LoggedShuffleboardTunableNumber m_ki;
  protected LoggedShuffleboardTunableNumber m_kd;
  protected LoggedShuffleboardTunableNumber m_kMaxAcceleration;
  protected LoggedShuffleboardTunableNumber m_kMaxVelocity;

  protected PositionSubsystemState m_currentState = null;
  protected PositionSubsystemState m_desiredState = null;

  protected boolean m_hasBeenZeroed = false;
  protected boolean m_isZeroed = true;

  protected boolean isRunningToPosition = false;

  protected boolean isRunningToSetpoint = false;

  protected double m_arbFeedforward = 0;

  protected TalonFXPositionSubsystem(final TalonFXPositionSubsystemConstants constants) {

    m_constants = constants;

    m_currentState = m_constants.kInitialState;
    m_desiredState = m_constants.kInitialState;

    m_master = new TalonFX(m_constants.kMasterConstants.kID);
    m_master.getConfigurator().apply(CtreUtils.generateMastorTalonFXConfig(m_constants.kMasterConstants));

    m_slaves = new TalonFX[m_constants.kSlaveConstants.length];

    for (int i = 0; i < m_constants.kSlaveConstants.length; i++) {
      m_slaves[i] = new TalonFX(m_constants.kSlaveConstants[i].kID);
      m_slaves[i].getConfigurator().apply(CtreUtils.generateMastorTalonFXConfig(m_constants.kSlaveConstants[i]));
      m_slaves[i].setControl(new StrictFollower(m_constants.kMasterConstants.kID));
    }

    // m_kp =
    //     new LoggedShuffleboardTunableNumber(
    //         m_constants.kName + " p",
    //         m_constants.kKp,
    //         RobotContainer.mechTuningTab,
    //         BuiltInWidgets.kTextView,
    //         Map.of("min", 0),
    //         0,
    //         m_constants.kSubsystemType.ordinal());

    // m_ki =
    //     new LoggedShuffleboardTunableNumber(
    //         m_constants.kName + " i",
    //         m_constants.kKi,
    //         RobotContainer.mechTuningTab,
    //         BuiltInWidgets.kTextView,
    //         Map.of("min", 0),
    //         1,
    //         m_constants.kSubsystemType.ordinal());

    // m_kd =
    //     new LoggedShuffleboardTunableNumber(
    //         m_constants.kName + " d",
    //         m_constants.kKd,
    //         RobotContainer.mechTuningTab,
    //         BuiltInWidgets.kTextView,
    //         Map.of("min", 0),
    //         2,
    //         m_constants.kSubsystemType.ordinal());

    // m_kMaxAcceleration =
    //     new LoggedShuffleboardTunableNumber(
    //         m_constants.kName + " Max Acceleration",
    //         m_constants.kMaxAcceleration,
    //         RobotContainer.mechTuningTab,
    //         BuiltInWidgets.kTextView,
    //         Map.of("min", 0),
    //         3,
    //         m_constants.kSubsystemType.ordinal());

    // m_kMaxVelocity =
    //     new LoggedShuffleboardTunableNumber(
    //         m_constants.kName + " Max Velocity",
    //         m_constants.kMaxVelocity,
    //         RobotContainer.mechTuningTab,
    //         BuiltInWidgets.kTextView,
    //         Map.of("min", 0),
    //         4,
    //         m_constants.kSubsystemType.ordinal());
    setName(m_constants.kName);
  }

  public void runToSetpoint() {

    if (!isRunningToPosition) {
      m_master.setControl(new MotionMagicVoltage(m_desiredState.getPosition(), false, m_arbFeedforward, m_constants.kDefaultSlot, true, false, false));
      isRunningToPosition = true;
    }
    if (m_currentState != m_constants.kTransitionState)
      m_currentState = m_constants.kTransitionState;

    m_constants.kTransitionState.setPosition(getPosition());
    m_constants.kTransitionState.setVelocity(getVelocity());

    if (atSetpoint()) {
      m_currentState = m_desiredState;
      m_constants.kManualState.setPosition(0);
      isRunningToPosition = false;
      isRunningToSetpoint = false;
    }
  }

  public void manualControl() {
    double m_throttle = 0;

    switch (m_constants.kManualControlMode) {
      case BUMPERS:
         m_throttle =
          RobotContainer.m_operatorController.getHID().getLeftBumper()
              ? 1
              : 0 + (RobotContainer.m_operatorController.getHID().getRightBumper() ? -1 : 0);
      case LEFT_X:
        m_throttle = -RobotContainer.m_driverController.getLeftX();
      case LEFT_Y:
        m_throttle = -RobotContainer.m_driverController.getLeftY();
      case RIGHT_X:
        m_throttle = -RobotContainer.m_driverController.getRightX();
      case RIGHT_Y:
        m_throttle = -RobotContainer.m_driverController.getRightY();
      case TRIGGERS:
        m_throttle =
          RobotContainer.m_driverController.getRightTriggerAxis()
              - RobotContainer.m_driverController.getLeftTriggerAxis();
    }

    m_throttle = MathUtil.applyDeadband(m_throttle, m_constants.kManualDeadBand);

    if (m_currentState != m_constants.kManualState)
      m_constants.kManualState.setPosition(getPosition());

    if (Math.abs(m_throttle) > 0 && m_currentState != m_constants.kTransitionState) {
      m_desiredState = m_constants.kManualState;
      m_currentState = m_constants.kManualState;

      m_throttle *= m_constants.kManualMultiplier;

      m_constants.kManualState.setPosition(m_constants.kManualState.getPosition() + m_throttle);
      m_constants.kManualState.setPosition(
          MathUtil.clamp(
              m_constants.kManualState.getPosition(),
              m_constants.kMinPosition,
              m_constants.kMaxPosition));
    }
  }

  public void holdPosition() {

    m_master.setControl(new PositionVoltage(m_desiredState.getPosition(), m_desiredState.getVelocity(), false, m_arbFeedforward, m_constants.kDefaultSlot, true, false, false));
  }

  public PositionSubsystemState getCurrentState() {
    return m_currentState;
  }

  public void setFeedforward(double feedforward) {
    m_arbFeedforward = feedforward;
  }

  public void setZeroed(boolean zeroed) {
    m_isZeroed = zeroed;
  }

  public void setDesiredState(PositionSubsystemState desiredState) {
    m_desiredState = desiredState;
    isRunningToSetpoint = true;
  }

  public boolean atSetpoint() {
    return Math.abs(m_desiredState.getPosition() - getPosition()) <= m_constants.kSetpointTolerance;
  }

  public double getPosition() {
    return RobotBase.isReal() ? m_master.getPosition().getValue() : m_currentState.getPosition();
  }

  public double getVelocity() {
    return RobotBase.isReal() ? m_master.getVelocity().getValue() : m_currentState.getVelocity();
  }

  public PositionSubsystemType getSubsystemType() {
    return m_constants.kSubsystemType;
  }

  @Override
  public void periodic() {

    if (m_isZeroed && !m_hasBeenZeroed) m_hasBeenZeroed = true;

    // System.out.println(isRunningToPosition);

    if (!isRunningToSetpoint) {
      holdPosition();
    } else {
      runToSetpoint();
    }

    subsystemPeriodic();

    outputTelemetry();

    // if (Constants.kTuningMode) {
    //   m_pidController.setP(m_kp.get(), m_constants.kDefaultSlot);
    //   m_pidController.setI(m_ki.get(), m_constants.kDefaultSlot);
    //   m_pidController.setD(m_kd.get(), m_constants.kDefaultSlot);
    //   m_profile =
    //       new TrapezoidProfile(
    //           new TrapezoidProfile.Constraints(m_kMaxVelocity.get(), m_kMaxAcceleration.get()));
    // }

    Logger.recordOutput(
        m_constants.kName + "/Encoder Position", getPosition()); // Current position of encoders
    Logger.recordOutput(m_constants.kName + "/Encoder Velocity", getVelocity()); // Encoder Velocity
    Logger.recordOutput(
        m_constants.kName + "/Trapezoid Desired Position",
        m_currentState.getPosition()); // Desired position of trapezoid profile
    Logger.recordOutput(
        m_constants.kName + "/Trapezoid Desired Velocity",
        m_currentState.getVelocity()); // Desired position of trapezoid profile
    Logger.recordOutput(
        m_constants.kName + "/Desired Position", m_desiredState.getPosition()); // Desired position
    Logger.recordOutput(
        m_constants.kName + "/Current State", m_currentState.getName()); // Current State
    Logger.recordOutput(
        m_constants.kName + "/Desired State", m_desiredState.getName()); // Current State
    Logger.recordOutput(m_constants.kName + "/At Setpoint", atSetpoint()); // Is at setpoint
  }

  public abstract void subsystemPeriodic();

  public abstract void outputTelemetry();
}
