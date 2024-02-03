// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.templates;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.lib.utilities.LoggedShuffleboardTunableNumber;
import frc.lib.utilities.ShuffleboardButton;

import java.util.Map;
import org.littletonrobotics.junction.Logger;

public abstract class PositionSubsystem extends SubsystemBase {

  public PositionSubsystemConstants m_constants;

  protected final SparkPIDController m_pidController;

  protected final CANSparkBase m_leader;
  protected final CANSparkBase[] m_followers;
  protected final RelativeEncoder m_encoder;

  protected TrapezoidProfile m_profile;
  protected TrapezoidProfile m_unzeroedProfile;
  protected TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  protected TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  protected double m_profileStartPosition = 0;
  protected double m_profileStartVelocity = 0;

  protected LoggedShuffleboardTunableNumber m_kp;
  protected LoggedShuffleboardTunableNumber m_ki;
  protected LoggedShuffleboardTunableNumber m_kd;
  protected LoggedShuffleboardTunableNumber m_kMaxAcceleration;
  protected LoggedShuffleboardTunableNumber m_kMaxVelocity;
  protected LoggedShuffleboardTunableNumber m_tuningPosition;
  protected ShuffleboardButton m_button;

  protected PositionSubsystemState m_currentState = null;
  protected PositionSubsystemState m_desiredState = null;

  protected boolean m_hasBeenZeroed = false;
  protected boolean m_isZeroed = true;

  protected double m_profileStartTime = -1;

  protected double m_arbFeedforward = 0;

  protected PositionSubsystem(final PositionSubsystemConstants constants) {

    m_constants = constants;

    m_currentState = m_constants.kInitialState;
    m_desiredState = m_constants.kInitialState;


    if (m_constants.kLeaderConstants.kRevMotorType == RevMotorType.CAN_SPARK_MAX) {
      m_leader =
        new CANSparkMax(m_constants.kLeaderConstants.kID, m_constants.kLeaderConstants.kMotorType);
    } else {
      m_leader =
        new CANSparkFlex(m_constants.kLeaderConstants.kID, m_constants.kLeaderConstants.kMotorType);
    }

    m_leader.setIdleMode(m_constants.kLeaderConstants.kIdleMode);
    m_leader.setSmartCurrentLimit(m_constants.kLeaderConstants.kCurrentLimit);

    m_pidController = m_leader.getPIDController();
    m_pidController.setP(m_constants.kLeaderConstants.kKp, m_constants.kDefaultSlot);
    m_pidController.setI(m_constants.kLeaderConstants.kKi, m_constants.kDefaultSlot);
    m_pidController.setD(m_constants.kLeaderConstants.kKd, m_constants.kDefaultSlot);

    m_encoder = m_leader.getEncoder();
    m_encoder.setPosition(m_constants.kHomePosition);
    m_encoder.setPositionConversionFactor(m_constants.kPositionConversionFactor);

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
      m_followers[i].follow(m_leader);
      m_followers[i].burnFlash();
    }

    if (m_followers.length > 0) m_leader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);

    m_kp =
        new LoggedShuffleboardTunableNumber(
            m_constants.kSubsystemName + " p",
            m_constants.kLeaderConstants.kKp,
            RobotContainer.positionMechTuningTab,
            BuiltInWidgets.kTextView,
            Map.of("min", 0),
            0,
            m_constants.kSubsystemType.ordinal());

    m_ki =
        new LoggedShuffleboardTunableNumber(
            m_constants.kSubsystemName + " i",
            m_constants.kLeaderConstants.kKi,
            RobotContainer.positionMechTuningTab,
            BuiltInWidgets.kTextView,
            Map.of("min", 0),
            1,
            m_constants.kSubsystemType.ordinal());

    m_kd =
        new LoggedShuffleboardTunableNumber(
            m_constants.kSubsystemName + " d",
            m_constants.kLeaderConstants.kKd,
            RobotContainer.positionMechTuningTab,
            BuiltInWidgets.kTextView,
            Map.of("min", 0),
            2,
            m_constants.kSubsystemType.ordinal());

    m_kMaxAcceleration =
        new LoggedShuffleboardTunableNumber(
            m_constants.kSubsystemName + " Max Acceleration",
            m_constants.kMaxAcceleration,
            RobotContainer.positionMechTuningTab,
            BuiltInWidgets.kTextView,
            Map.of("min", 0),
            3,
            m_constants.kSubsystemType.ordinal());

    m_kMaxVelocity =
        new LoggedShuffleboardTunableNumber(
            m_constants.kSubsystemName + " Max Velocity",
            m_constants.kMaxVelocity,
            RobotContainer.positionMechTuningTab,
            BuiltInWidgets.kTextView,
            Map.of("min", 0),
            4,
            m_constants.kSubsystemType.ordinal());
    m_tuningPosition =
        new LoggedShuffleboardTunableNumber(
            m_constants.kSubsystemName + " Set Position",
            0,
            RobotContainer.positionMechTuningTab,
            BuiltInWidgets.kTextView,
            null,
            5,
            m_constants.kSubsystemType.ordinal());


    m_profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                m_constants.kMaxVelocity, m_constants.kMaxAcceleration));
    m_unzeroedProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                m_constants.kMaxVelocity * .01, m_constants.kMaxAcceleration * .01));

    setName(m_constants.kSubsystemName);
  }

  public void runToSetpoint() {

    if (m_hasBeenZeroed) {
      m_setpoint =
          m_profile.calculate(
              Timer.getFPGATimestamp() - m_profileStartTime,
              new TrapezoidProfile.State(m_profileStartPosition, m_profileStartVelocity),
              new TrapezoidProfile.State(m_desiredState.getPosition(), 0));
    } else {
      m_setpoint =
          m_unzeroedProfile.calculate(
              Timer.getFPGATimestamp() - m_profileStartTime,
              new TrapezoidProfile.State(m_profileStartPosition, m_profileStartVelocity),
              new TrapezoidProfile.State(m_desiredState.getPosition(), 0));
    }

    m_pidController.setReference(
        m_setpoint.position,
        ControlType.kPosition,
        m_constants.kDefaultSlot,
        m_arbFeedforward,
        ArbFFUnits.kVoltage);

    if (m_currentState != m_constants.kTransitionState)
      m_currentState = m_constants.kTransitionState;

    m_constants.kTransitionState.setPosition(m_setpoint.position);
    m_constants.kTransitionState.setVelocity(m_setpoint.velocity);

    if (m_setpoint.position == m_desiredState.getPosition()) {
      m_profileStartTime = -1;
      m_currentState = m_desiredState;
      m_constants.kManualState.setPosition(0);
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

    if (Math.abs(m_throttle) > 0 && m_profileStartTime == -1) {
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
    m_pidController.setReference(
        m_currentState.getPosition(),
        ControlType.kPosition,
        m_constants.kDefaultSlot,
        m_arbFeedforward,
        ArbFFUnits.kVoltage);
  }

  public PositionSubsystemState getCurrentState() {
    return m_currentState;
  }

  public void setFeedforward(double feedforward) {
    m_arbFeedforward = feedforward;
  }

  public void zero(double position) {
    m_isZeroed = true;
    m_encoder.setPosition(position);
  }

  public void setDesiredState(PositionSubsystemState desiredState, boolean useMotionProfile) {
    m_desiredState = desiredState;
    m_profileStartPosition = getPosition();
    m_profileStartVelocity = getVelocity();
    if (useMotionProfile) m_profileStartTime = Timer.getFPGATimestamp();
  }

  public boolean atSetpoint() {
    return Math.abs(m_desiredState.getPosition() - getPosition()) <= m_constants.kSetpointTolerance;
  }

  public double getPosition() {
    return RobotBase.isReal() ? m_encoder.getPosition() : m_currentState.getPosition();
  }

  public double getVelocity() {
    return RobotBase.isReal() ? m_encoder.getVelocity() : m_currentState.getVelocity();
  }

  public PositionSubsystemType getSubsystemType() {
    return m_constants.kSubsystemType;
  }

  @Override
  public void periodic() {

    if (m_isZeroed && !m_hasBeenZeroed) m_hasBeenZeroed = true;

    if (!(m_profileStartTime == -1)) {
      runToSetpoint();
    } else if (m_currentState == m_constants.kManualState) {
      holdPosition();
    }

    subsystemPeriodic();

    outputTelemetry();

    if (Constants.kTuningMode) {
      if (RobotContainer.m_applyPositionMechConfigs.getValue()) {
        m_pidController.setP(m_kp.get(), m_constants.kDefaultSlot);
        m_pidController.setI(m_ki.get(), m_constants.kDefaultSlot);
        m_pidController.setD(m_kd.get(), m_constants.kDefaultSlot);
        m_leader.burnFlash();
        m_profile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(m_kMaxVelocity.get(), m_kMaxAcceleration.get()));

      }

      if (RobotContainer.m_goToPosition.getValue()) {
        if (m_currentState != m_constants.kManualState)
        m_constants.kManualState.setPosition(getPosition());

        if (m_profileStartTime == -1) {
          m_desiredState = m_constants.kManualState;
          m_currentState = m_constants.kManualState;

          m_constants.kManualState.setPosition(m_tuningPosition.get());
          m_constants.kManualState.setPosition(
              MathUtil.clamp(
                  m_constants.kManualState.getPosition(),
                  m_constants.kMinPosition,
                  m_constants.kMaxPosition));
        }
      }
    }

    Logger.recordOutput(
        m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Encoder Position", getPosition()); // Current position of encoders
    Logger.recordOutput(m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Encoder Velocity", getVelocity()); // Encoder Velocity
    Logger.recordOutput(
        m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Trapezoid Desired Position",
        m_currentState.getPosition()); // Desired position of trapezoid profile
    Logger.recordOutput(
        m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Trapezoid Desired Velocity",
        m_currentState.getVelocity()); // Desired position of trapezoid profile
    Logger.recordOutput(
        m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Desired Position", m_desiredState.getPosition()); // Desired position
    Logger.recordOutput(
        m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Current State", m_currentState.getName()); // Current State
    Logger.recordOutput(
        m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Desired State", m_desiredState.getName()); // Current State
    Logger.recordOutput(m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/At Setpoint", atSetpoint()); // Is at setpoint
  }

  public abstract void subsystemPeriodic();

  public abstract void outputTelemetry();

  public enum PositionSubsystemType {
    LAUNCHER_WRIST,
    FALCON_TESTING
  }

  public interface PositionSubsystemState {
    String getName();

    double getPosition();

    double getVelocity();

    void setPosition(double position);

    void setVelocity(double velocity);
  }
}

// EXAMPLE POSITION SUBSYSTEM IMPLEMENTATION


// public class ExamplePositionSubsystem extends PositionSubsystem {


//     private static ExamplePositionSubsystem m_instance = null;

//     public ExamplePositionSubsystem(PositionSubsystemConstants constants) {
//         super(constants);
//     }

//     public static ExamplePositionSubsystem getInstance() {
//         if (m_instance == null) {
//             m_instance = new ExamplePositionSubsystem(ExampleConstants.kExamplePositionSubsystemConstants);
//         }

//         return m_instance;
//     }

//     @Override
//     public void subsystemPeriodic() {
//     }

//     @Override
//     public void outputTelemetry() {}

//     public enum ExamplePositionSubsystemState implements PositionSubsystemState {
//         DOWN(0, 0, "Down"),
//         UP(45, 0, "Up"),
//         TRANSITION(0, 0, "Transition"),
//         MANUAL(0, 0, "Manual");
    
//         private double position;
//         private double velocity;
//         private String name;
    
//         private ExamplePositionSubsystemState(double position, double velocity, String name) {
//           this.position = position;
//           this.velocity = velocity;
//           this.name = name;
//         }

//         @Override
//         public String getName() {
//             return name;
//         }

//         @Override
//         public double getVelocity() {
//             return velocity;
//         }

//         @Override
//         public void setVelocity(double velocity) {
//             this.velocity = velocity;
//         }

//         @Override
//         public double getPosition() {
//             return position;
//         }

//         @Override
//         public void setPosition(double position) {
//             this.position = position;
//         }
//     }
    
// }
