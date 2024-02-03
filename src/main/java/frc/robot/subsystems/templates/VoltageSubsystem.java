// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.templates;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.VoltageSubsystemConstants;

public abstract class VoltageSubsystem extends SubsystemBase {

  public VoltageSubsystemConstants m_constants;

  protected final CANSparkBase m_leader;
  protected final CANSparkBase[] m_followers;
  protected final RelativeEncoder m_encoder;

  protected VoltageSubsystemState m_currentState = null;

  protected VoltageSubsystem(final VoltageSubsystemConstants constants) {

    m_constants = constants;

    m_currentState = m_constants.kInitialState;

    if (m_constants.kLeaderConstants.kRevMotorType == RevMotorType.CAN_SPARK_MAX) {
      m_leader =
        new CANSparkMax(m_constants.kLeaderConstants.kID, m_constants.kLeaderConstants.kMotorType);
    } else {
      m_leader =
        new CANSparkFlex(m_constants.kLeaderConstants.kID, m_constants.kLeaderConstants.kMotorType);
    }
    m_leader.setIdleMode(m_constants.kLeaderConstants.kIdleMode);
    m_leader.setSmartCurrentLimit(m_constants.kLeaderConstants.kCurrentLimit);
    m_leader.setInverted(m_constants.kLeaderConstants.kInverted);
    m_leader.burnFlash();

    if (m_constants.kFollowerConstants.length > 0) {
      if (m_constants.kFollowerConstants[0].kRevMotorType == RevMotorType.CAN_SPARK_MAX) {
        m_followers = new CANSparkMax[m_constants.kFollowerConstants.length];
      } else {
        m_followers = new CANSparkFlex[m_constants.kFollowerConstants.length];
      }
    } else {
      m_followers = new CANSparkBase[0];
    }

    for (int i = 0; i < m_constants.kFollowerConstants.length; i++) {
      if (m_constants.kFollowerConstants[0].kRevMotorType == RevMotorType.CAN_SPARK_MAX) {
        m_followers[i] =
          new CANSparkMax(
              m_constants.kFollowerConstants[i].kID, m_constants.kFollowerConstants[i].kMotorType);
      } else {
        m_followers[i] =
          new CANSparkFlex(
              m_constants.kFollowerConstants[i].kID, m_constants.kFollowerConstants[i].kMotorType);
      }
      m_followers[i].setIdleMode(m_constants.kFollowerConstants[i].kIdleMode);
      m_followers[i].setSmartCurrentLimit(m_constants.kFollowerConstants[i].kCurrentLimit);
      m_followers[i].follow(m_leader, m_constants.kFollowerConstants[i].kInverted);
      m_followers[i].burnFlash();
    }

    m_encoder = m_leader.getEncoder();

    setName(m_constants.kSubsystemName);
  }

  public VoltageSubsystemState getCurrentState() {
    return m_currentState;
  }

  public void setState(VoltageSubsystemState desiredState) {
    m_currentState = desiredState;
    m_leader.setVoltage(m_currentState.getVoltage());
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public double getVolts() {
    return RobotBase.isReal() ? m_leader.getBusVoltage() : m_currentState.getVoltage();
  }

  public VoltageSubsystemType getSubsystemType() {
    return m_constants.kSubsystemType;
  }

  @Override
  public void periodic() {
    subsystemPeriodic();
    outputTelemetry();
  }

  public abstract void subsystemPeriodic();

  public abstract void outputTelemetry();

  public enum VoltageSubsystemType {
    LAUNCHER_HOLD
  }

  public interface VoltageSubsystemState {
    String getName();

    double getVoltage();
  }
}


// public class ExampleVoltageSubsystem extends VoltageSubsystem {

//     private static ExampleVoltageSubsystem m_instance = null;

//     protected ExampleVoltageSubsystem(VoltageSubsystemConstants constants) {
//         super(constants);
//     }

//     public static ExampleVoltageSubsystem getInstance() {
//         if (m_instance == null) {
//             m_instance = new ExampleVoltageSubsystem(ExampleConstants.kExampleVoltageSubsystemConstants);
//         }

//         return m_instance;
//     }


//     @Override
//     public void subsystemPeriodic() {}

//     @Override
//     public void outputTelemetry() {}

//     public enum ExampleVoltageSubsystemState implements VoltageSubsystemState {
//         OFF(0, "Off"),
//         FEEDING(3, "Up"),
//         OUTTAKING(-3, "Outtaking");
    
//         private double voltage;
//         private String name;
    
//         private ExampleVoltageSubsystemState(double voltage, String name) {
//           this.voltage = voltage;
//           this.name = name;
//         }

//         @Override
//         public String getName() {
//             return name;
//         }

//         @Override
//         public double getVoltage() {
//             return voltage;
//         }
//     }
    
// }

