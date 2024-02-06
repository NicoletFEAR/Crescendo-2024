// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.templates;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.VelocitySubsystemConstants;
import frc.lib.utilities.LoggedShuffleboardTunableNumber;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

public abstract class VelocitySubsystem extends SubsystemBase {

  public VelocitySubsystemConstants m_constants;

  protected final CANSparkBase[] m_motors;
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

    if (m_constants.kMotorConstants[0].kRevMotorType == RevMotorType.CAN_SPARK_MAX) {
      m_motors = new CANSparkMax[m_constants.kMotorConstants.length];
    } else {
      m_motors = new CANSparkFlex[m_constants.kMotorConstants.length];
    }

    m_encoders = new RelativeEncoder[m_motors.length];
    m_pidControllers = new SparkPIDController[m_motors.length];
    m_arbFeedforward = new double[m_motors.length];

    m_kp = new LoggedShuffleboardTunableNumber[m_motors.length];
    m_ki = new LoggedShuffleboardTunableNumber[m_motors.length];
    m_kd = new LoggedShuffleboardTunableNumber[m_motors.length];
    m_tuningVelocity = new LoggedShuffleboardTunableNumber[m_motors.length];

    for (int i = 0; i < m_constants.kMotorConstants.length; i++) {
      if (m_constants.kMotorConstants[i].kRevMotorType == RevMotorType.CAN_SPARK_MAX) {
        m_motors[i] =
          new CANSparkMax(m_constants.kMotorConstants[i].kID, m_constants.kMotorConstants[i].kMotorType);
      } else {
        m_motors[i] =
          new CANSparkFlex(m_constants.kMotorConstants[i].kID, m_constants.kMotorConstants[i].kMotorType);
      }
      m_motors[i].setIdleMode(m_constants.kMotorConstants[i].kIdleMode);
      m_motors[i].setSmartCurrentLimit(m_constants.kMotorConstants[i].kCurrentLimit);
      m_motors[i].setInverted(m_constants.kMotorConstants[i].kInverted);
      m_motors[i].enableVoltageCompensation(12.6);

      m_encoders[i] = m_motors[i].getEncoder();
      m_encoders[i].setVelocityConversionFactor(m_constants.kVelocityConversionFactor);

      m_pidControllers[i] = m_motors[i].getPIDController();
      m_pidControllers[i].setP(m_constants.kMotorConstants[i].kKp);
      m_pidControllers[i].setI(m_constants.kMotorConstants[i].kKi);
      m_pidControllers[i].setD(m_constants.kMotorConstants[i].kKd);
      m_pidControllers[i].setFF(m_constants.kMotorConstants[i].kKff);

      m_motors[i].burnFlash();


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

    setName(m_constants.kSubsystemName);
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

    if (atSetpoint()) {
        m_currentState = m_desiredState;
    } else{
        m_currentState = m_constants.kTransitionState;
        m_currentState.setVelocity(getVelocity());
    }

    subsystemPeriodic();

    outputTelemetry();

    if (Constants.kTuningMode) {
      if (RobotContainer.m_applyVelocityMechConfigs.getValue()) {
        for (int i = 0; i < m_constants.kMotorConstants.length; i++) {
          m_pidControllers[i].setP(m_constants.kMotorConstants[i].kKp);
          m_pidControllers[i].setI(m_constants.kMotorConstants[i].kKi);
          m_pidControllers[i].setD(m_constants.kMotorConstants[i].kKd);
          m_pidControllers[i].setFF(m_constants.kMotorConstants[i].kKff);
          m_motors[i].burnFlash();
        }
      }

      if (RobotContainer.m_goToVelocity.getValue()) {
        double[] speeds = new double[m_motors.length];
        for (int i = 0; i < speeds.length; i++) {
          speeds[i] = m_tuningVelocity[i].get();
        }

        m_constants.kManualState.setVelocity(speeds);
        setDesiredState(m_constants.kManualState);
        m_currentState = m_constants.kManualState;
      }
    }

    for (int i = 0; i < m_motors.length; i++) {
      Logger.recordOutput(m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/" + m_constants.kMotorConstants[i].kName + "/Encoder Velocity", getVelocity()[i]); // Encoder Velocity
      Logger.recordOutput(m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/" + m_constants.kMotorConstants[i].kName + "/Desired Velocity", m_desiredState.getVelocity()[i]); // Desired position
    }

    Logger.recordOutput(
      m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Current State", m_currentState.getName()); // Current State
    Logger.recordOutput(
      m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Desired State", m_desiredState.getName()); // Current State
    Logger.recordOutput(m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/At Setpoint", atSetpoint()); // Is at setpoint
  }

  public abstract void subsystemPeriodic();

  public abstract void outputTelemetry();

  public enum VelocitySubsystemType {
    LAUNCHER_FLYWHEEL,
  }

  public interface VelocitySubsystemState {
    String getName();

    double[] getVelocity();

    void setVelocity(double[] velocity);
  }
}


// EXAMPLE IMPLEMENTATION

// public class ExampleVelocitySubsystem extends VelocitySubsystem {

//     private static ExampleVelocitySubsystem m_instance = null;

//     public ExampleVelocitySubsystem(VelocitySubsystemConstants constants) {
//         super(constants);
//     }

//     public static ExampleVelocitySubsystem getInstance() {
//         if (m_instance == null) {
//             m_instance = new ExampleVelocitySubsystem(LauncherConstants.kLauncherFlywheelConstants);
//         }

//         return m_instance;
//     }

//     @Override
//     public void subsystemPeriodic() {}

//     @Override
//     public void outputTelemetry() {}

//     public enum ExampleVelocitySubsystemState implements VelocitySubsystemState {
//         OFF(new double[] {0, 0}, "Off"),
//         IDLE(new double[] {-3500, -3500}, "Idle"),
//         FAST(new double[] {-4000, -4000}, "Fast"),
//         TRANSITION(new double[] {0, 0}, "Transition"),
//         FIELD_BASED_VELOCITY(new double[] {0, 0}, "Field Based Velocity"),
//         RUNNING(new double[] {-5500, -5500}, "Running"),
//         MANUAL(new double[] {0, 0}, "Manual");
    
//         private double[] velocity;
//         private String name;
    
//         private ExampleVelocitySubsystemState(double[] velocity, String name) {
//           this.velocity = velocity;
//           this.name = name;
//         }

//         @Override
//         public String getName() {
//             return name;
//         }

//         @Override
//         public double[] getVelocity() {
//             return velocity;
//         }

//         @Override
//         public void setVelocity(double[] velocity) {
//             this.velocity = velocity;
//         }
//     }
    
// }