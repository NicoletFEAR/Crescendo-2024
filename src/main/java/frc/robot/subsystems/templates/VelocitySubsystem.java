// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.templates;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.templates.SubsystemConstants.VelocitySubsystemConstants;
import frc.lib.utilities.LoggedShuffleboardTunableNumber;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

public abstract class VelocitySubsystem extends SubsystemBase {

  public VelocitySubsystemConstants m_constants;

  protected final CANSparkMax[] m_motors;
  protected final RelativeEncoder[] m_encoders;
  protected final SparkPIDController[] m_pidControllers;

  protected LoggedShuffleboardTunableNumber[] m_kp;
  protected LoggedShuffleboardTunableNumber[] m_ki;
  protected LoggedShuffleboardTunableNumber[] m_kd;
  protected LoggedShuffleboardTunableNumber[] m_tuningVelocity;

  protected VelocitySubsystemState m_currentState = null;
  protected VelocitySubsystemState m_desiredState = null;

  protected double[] m_arbFeedforward;

  protected double simpos = 0;

  protected VelocitySubsystem(final VelocitySubsystemConstants constants) {

    m_constants = constants;

    m_currentState = m_constants.kInitialState;
    m_desiredState = m_constants.kInitialState;

    m_motors = new CANSparkMax[m_constants.kMotorConstants.length];
    m_encoders = new RelativeEncoder[m_motors.length];
    m_pidControllers = new SparkPIDController[m_motors.length];
    m_arbFeedforward = new double[m_motors.length];

    for (int i = 0; i < m_constants.kMotorConstants.length; i++) {
      m_motors[i] =
          new CANSparkMax(
              m_constants.kMotorConstants[i].kID, m_constants.kMotorConstants[i].kMotorType);
      m_motors[i].setIdleMode(m_constants.kMotorConstants[i].kIdleMode);
      m_motors[i].setSmartCurrentLimit(m_constants.kMotorConstants[i].kCurrentLimit);
      m_motors[i].setInverted(m_constants.kMotorConstants[i].kInverted);
      m_encoders[i] = m_motors[i].getEncoder();
      m_pidControllers[i] = m_motors[i].getPIDController();

      m_pidControllers[i].setP(m_constants.kMotorConstants[i].kKp);
      m_pidControllers[i].setI(m_constants.kMotorConstants[i].kKi);
      m_pidControllers[i].setD(m_constants.kMotorConstants[i].kKd);
      m_pidControllers[i].setFF(m_constants.kMotorConstants[i].kKff);

      m_motors[i].burnFlash();
    }

    // m_encoder.setVelocityConversionFactor(m_constants.kVelocityConversionFactor);

    m_kp = new LoggedShuffleboardTunableNumber[m_motors.length];
    m_ki = new LoggedShuffleboardTunableNumber[m_motors.length];
    m_kd = new LoggedShuffleboardTunableNumber[m_motors.length];
    m_tuningVelocity = new LoggedShuffleboardTunableNumber[m_motors.length];

    for (int i = 0; i < m_motors.length; i ++) {
      m_kp[i] =
          new LoggedShuffleboardTunableNumber(
              m_constants.kMotorConstants[i].kName + " p",
              m_constants.kMotorConstants[i].kKp,
              RobotContainer.velocityMechTuningTab,
              BuiltInWidgets.kTextView,
              Map.of("min", 0),
              0,
              i);

      m_ki[i] =
          new LoggedShuffleboardTunableNumber(
              m_constants.kMotorConstants[i].kName + " i",
              m_constants.kMotorConstants[i].kKi,
              RobotContainer.velocityMechTuningTab,
              BuiltInWidgets.kTextView,
              Map.of("min", 0),
              1,
              i);

      m_kd[i] =
          new LoggedShuffleboardTunableNumber(
              m_constants.kMotorConstants[i].kName + " d",
              m_constants.kMotorConstants[i].kKd,
              RobotContainer.velocityMechTuningTab,
              BuiltInWidgets.kTextView,
              Map.of("min", 0),
              2,
              i);
    m_tuningVelocity[i] =
        new LoggedShuffleboardTunableNumber(
            m_constants.kMotorConstants[i].kName + " Set Position",
            0,
            RobotContainer.velocityMechTuningTab,
            BuiltInWidgets.kTextView,
            null,
            3,
            i);
    }



    setName(m_constants.kName);
  }

  public VelocitySubsystemState getCurrentState() {
    return m_currentState;
  }

  public void setFeedforward(double[] feedforward) {
    m_arbFeedforward = feedforward;
  }

  public void setDesiredState(VelocitySubsystemState desiredState) {
    m_desiredState = desiredState;
    for (int i = 0; i < m_pidControllers.length; i++) {
      m_pidControllers[i].setReference(m_desiredState.getVelocity()[i], ControlType.kVelocity, m_constants.kDefaultSlot, m_arbFeedforward[i], ArbFFUnits.kVoltage);
    }
  }

  public boolean atSetpoint() {
    boolean output = true;

    for (int i = 0; i < getVelocity().length; i++) {
      if (Math.abs(m_desiredState.getVelocity()[i] - getVelocity()[i]) >= m_constants.kSetpointTolerance) {
        output = false;
        break;
      }
    }

    return output;
  }

  public double[] getVelocity() {
    double[] output = new double[m_encoders.length];

    for (int i = 0; i < output.length; i++) {
      output[i] = m_encoders[i].getVelocity();
    }

    return output;
  }

  public VelocitySubsystemType getSubsystemType() {
    return m_constants.kSubsystemType;
  }

  @Override
  public void periodic() {

    if (atSetpoint() && m_desiredState != m_constants.kManualState && m_currentState != m_desiredState) {
        m_currentState = m_desiredState;
    } else {
        m_currentState = m_constants.kTransitionState;
        m_currentState.setVelocity(getVelocity());
    }

    subsystemPeriodic();

    outputTelemetry();

    if (RobotContainer.m_applyVelocityMechConfigs.getValue()) {
      for (int i = 0; i < m_constants.kMotorConstants.length; i++) {
        m_pidControllers[i].setP(m_constants.kMotorConstants[i].kKp);
        m_pidControllers[i].setI(m_constants.kMotorConstants[i].kKi);
        m_pidControllers[i].setD(m_constants.kMotorConstants[i].kKd);
        m_pidControllers[i].setFF(m_constants.kMotorConstants[i].kKff);
        m_motors[i].burnFlash();
      }

      double[] speeds = new double[m_motors.length];
      for (int i = 0; i < speeds.length; i++) {
        speeds[i] = m_tuningVelocity[i].get();
      }

      m_constants.kManualState.setVelocity(speeds);
      setDesiredState(m_constants.kManualState);
      m_currentState = m_constants.kManualState;
    }

    for (int i = 0; i < m_motors.length; i++) {
      Logger.recordOutput(m_constants.kName + "/" + m_constants.kMotorConstants[i].kName + "/Encoder Velocity", getVelocity()[i]); // Encoder Velocity
      Logger.recordOutput(m_constants.kName + "/" + m_constants.kMotorConstants[i].kName + "/Desired Velocity", m_desiredState.getVelocity()[i]); // Desired position
    }

    Logger.recordOutput(
        m_constants.kName + "/Current State", m_currentState.getName()); // Current State
    Logger.recordOutput(
        m_constants.kName + "/Desired State", m_desiredState.getName()); // Current State
    Logger.recordOutput(m_constants.kName + "/At Setpoint", atSetpoint()); // Is at setpoint
  }

  public abstract void subsystemPeriodic();

  public abstract void outputTelemetry();

  public enum VelocitySubsystemType {
    LAUNCHER_FLYWHEEL
  }

  public interface VelocitySubsystemState {
    String getName();

    double[] getVelocity();

    void setVelocity(double[] velocity);
  }
}
