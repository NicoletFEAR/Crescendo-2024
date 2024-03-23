// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.templates;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.templates.SubsystemConstants.MultiMotorPositionSubsystemConstants;
import frc.lib.templates.SubsystemConstants.RevMotorType;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public abstract class MultiMotorPositionSubsystem extends SubsystemBase {

  public MultiMotorPositionSubsystemConstants m_constants;

  protected final CANSparkBase[] m_motors;
  protected final RelativeEncoder[] m_encoders;
  protected final SparkPIDController[] m_pidControllers;

  protected TrapezoidProfile m_profile;
  protected TrapezoidProfile m_unzeroedProfile;
  protected TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  protected TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  protected double[] m_profileStartPosition;
  protected double[] m_profileStartVelocity;

  protected MultiMotorPositionSubsystemState m_currentState = null;
  protected MultiMotorPositionSubsystemState m_desiredState = null;

  protected boolean m_hasBeenZeroed = false;
  protected boolean m_isZeroed = true;

  protected double m_profileStartTime = -1;

  protected double m_arbFeedforward = 0;

  protected MultiMotorPositionSubsystem(final MultiMotorPositionSubsystemConstants constants) {

    m_constants = constants;

    m_currentState = m_constants.kInitialState;
    m_desiredState = m_constants.kInitialState;

    if (m_constants.kMotorConstants[0].kRevMotorType == RevMotorType.CAN_SPARK_MAX) {
      m_motors = new CANSparkMax[m_constants.kMotorConstants.length];
    } else {
      m_motors = new CANSparkFlex[m_constants.kMotorConstants.length];
    }
    
    m_pidControllers = new SparkPIDController[m_motors.length];
    m_encoders = new RelativeEncoder[m_motors.length];
    m_profileStartPosition = new double[m_motors.length];
    m_profileStartVelocity = new double[m_motors.length];

    for (int i = 0; i < m_motors.length; i++) {
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

      m_pidControllers[i] = m_motors[i].getPIDController();
      m_pidControllers[i].setP(m_constants.kMotorConstants[i].kKp, m_constants.kDefaultSlot);
      m_pidControllers[i].setI(m_constants.kMotorConstants[i].kKi, m_constants.kDefaultSlot);
      m_pidControllers[i].setD(m_constants.kMotorConstants[i].kKd, m_constants.kDefaultSlot);

      m_encoders[i] = m_motors[i].getEncoder();
      m_encoders[i].setPosition(m_constants.kHomePosition);
      m_encoders[i].setPositionConversionFactor(m_constants.kPositionConversionFactor);

      m_motors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
      m_motors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
      m_motors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
      m_motors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
      m_motors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20);
      m_motors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200);
      m_motors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);

      try {
        Thread.sleep(200);
      } catch (Exception e) {}
      m_motors[i].burnFlash();
    }

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

    if (m_currentState != m_constants.kTransitionState) m_currentState = m_constants.kTransitionState;

    for (int i = 0; i < m_motors.length; i++) {
      if (m_hasBeenZeroed) {
        m_setpoint =
            m_profile.calculate(
                Timer.getFPGATimestamp() - m_profileStartTime,
                new TrapezoidProfile.State(m_profileStartPosition[i], m_profileStartVelocity[i]),
                new TrapezoidProfile.State(m_desiredState.getPosition()[i], 0));
      } else {
        m_setpoint =
            m_unzeroedProfile.calculate(
                Timer.getFPGATimestamp() - m_profileStartTime,
                new TrapezoidProfile.State(m_profileStartPosition[i], m_profileStartVelocity[i]),
                new TrapezoidProfile.State(m_desiredState.getPosition()[i], 0));
      }

      m_pidControllers[i].setReference(
          m_setpoint.position,
          ControlType.kPosition,
          m_constants.kDefaultSlot,
          m_arbFeedforward,
          ArbFFUnits.kVoltage);



      m_constants.kTransitionState.setPosition(m_setpoint.position, i);
      m_constants.kTransitionState.setVelocity(m_setpoint.velocity, i);

      if (m_setpoint.position == m_desiredState.getPosition()[i]) {
        m_profileStartTime = -1;
        m_currentState = m_desiredState;
        m_constants.kManualState.setPosition(0, i);
      }
    }
  }

  public abstract void manualControl();

  // Example implementation for one motor
  // Really on a per mech basis, you have to override this if you want to use it
  // {
    // double m_throttle = 0;

    // switch (m_constants.kManualControlMode) {
    //   case BUMPERS:
    //      m_throttle =
    //       RobotContainer.m_operatorController.getHID().getLeftBumper()
    //           ? 1
    //           : 0 + (RobotContainer.m_operatorController.getHID().getRightBumper() ? -1 : 0);
    //   case LEFT_X:
    //     m_throttle = -RobotContainer.m_driverController.getLeftX();
    //   case LEFT_Y:
    //     m_throttle = -RobotContainer.m_driverController.getLeftY();
    //   case RIGHT_X:
    //     m_throttle = -RobotContainer.m_driverController.getRightX();
    //   case RIGHT_Y:
    //     m_throttle = -RobotContainer.m_driverController.getRightY();
    //   case TRIGGERS:
    //     m_throttle =
    //       RobotContainer.m_driverController.getRightTriggerAxis()
    //           - RobotContainer.m_driverController.getLeftTriggerAxis();
    // }

    // m_throttle = MathUtil.applyDeadband(m_throttle, m_constants.kManualDeadBand);

    // if (m_currentState != m_constants.kManualState)
    //   m_constants.kManualState.setPosition(getPosition());

    // if (Math.abs(m_throttle) > 0 && m_profileStartTime == -1) {
    //   m_desiredState = m_constants.kManualState;
    //   m_currentState = m_constants.kManualState;

    //   m_throttle *= m_constants.kManualMultiplier;

    //   m_constants.kManualState.setPosition(m_constants.kManualState.getPosition() + m_throttle);
    //   m_constants.kManualState.setPosition(
    //       MathUtil.clamp(
    //           m_constants.kManualState.getPosition(),
    //           m_constants.kMinPosition,
    //           m_constants.kMaxPosition));
    // }
  // }

  public void holdPosition() {
    for (int i = 0; i < m_motors.length; i++) {
      m_pidControllers[i].setReference(
          m_currentState.getPosition()[i],
          ControlType.kPosition,
          m_constants.kDefaultSlot,
          m_arbFeedforward,
          ArbFFUnits.kVoltage);
    }
  }

  public MultiMotorPositionSubsystemState getCurrentState() {
    return m_currentState;
  }

  public MultiMotorPositionSubsystemState getDesiredState() {
    return m_desiredState;
  }

  public void setFeedforward(double feedforward) {
    m_arbFeedforward = feedforward;
  }

  public void zero(double position) {
    m_isZeroed = true;
    for (RelativeEncoder encoder : m_encoders) {
      encoder.setPosition(position);
    }
    
  }

  public void setDesiredState(MultiMotorPositionSubsystemState desiredState, boolean useMotionProfile) {
    if (m_currentState != m_constants.kManualState) {
      for (int i = 0; i < m_motors.length; i++) {
        m_setpoint =
            m_profile.calculate(
                Timer.getFPGATimestamp() - m_profileStartTime,
                new TrapezoidProfile.State(m_profileStartPosition[i], m_profileStartVelocity[i]),
                new TrapezoidProfile.State(m_desiredState.getPosition()[i], 0));
        m_profileStartPosition[i] = m_setpoint.position;
        m_profileStartVelocity[i] = m_setpoint.velocity;
      }
    } else {
      m_profileStartPosition = getPosition();
      m_profileStartVelocity = new double[m_motors.length];
    }

    m_desiredState = desiredState;
    if (useMotionProfile) m_profileStartTime = Timer.getFPGATimestamp();
  }

  public boolean atSetpoint() {
    boolean output = false;

    for (int i = 0; i < m_motors.length; i++) {
      if (m_profileStartTime != -1 && Math.abs(m_desiredState.getPosition()[i] - m_setpoint.position) <= m_constants.kSetpointTolerance || m_currentState == m_constants.kManualState)  {
        output = Math.abs(m_desiredState.getPosition()[i] - getPosition()[i]) <= m_constants.kSetpointTolerance;
        break;
      } else if (m_profileStartTime == -1) {
        return true;
      }
    }

    return output;
  }

  public double[] getPosition() {
    if (Constants.kCurrentMode == Mode.REAL) {
      double[] output = new double[m_motors.length];

      for (int i = 0; i < output.length; i++) {
        output[i] = m_encoders[i].getPosition();
      }

      return output;
    } else {
      return m_currentState.getPosition();
    }
  }

  public double[] getVelocity() {
        if (Constants.kCurrentMode == Mode.REAL) {
      double[] output = new double[m_motors.length];

      for (int i = 0; i < output.length; i++) {
        output[i] = m_encoders[i].getVelocity();
      }

      return output;
    } else {
      return m_currentState.getVelocity();
    }
  }

  public MultiMotorPositionSubsystemType getSubsystemType() {
    return m_constants.kSubsystemType;
  }

  @Override
  public void periodic() {

    if (m_isZeroed && !m_hasBeenZeroed) m_hasBeenZeroed = true;

    if (!(m_profileStartTime == -1)) {
      runToSetpoint();
    }

    if (Constants.kInfoMode) {
      SmartDashboard.putString(m_constants.kSubsystemName + "/Current State", m_currentState.getName());
      SmartDashboard.putString(m_constants.kSubsystemName + "/Desired State", m_desiredState.getName());

      SmartDashboard.putNumber(m_constants.kSubsystemName + "/Current Position", getPosition()[0]);
      SmartDashboard.putNumber(m_constants.kSubsystemName + "/Desired Position", m_desiredState.getPosition()[0]);
    }
    
    subsystemPeriodic();
  }

  public abstract void subsystemPeriodic();

  public abstract void outputTelemetry();

  public enum MultiMotorPositionSubsystemType {
    ELEVATOR_LIFT
  }

  public interface MultiMotorPositionSubsystemState {
    String getName();

    double[] getPosition();

    double[] getVelocity();

    void setPosition(double position, int index);

    void setVelocity(double velocity, int index);
  }
}


// EXAMPLE SUBSYSTEM IMPLEMENTATION

// public class ExampleMultiMotorPositionSubsystem extends MultiMotorPositionSubsystem {

//     private static ExampleMultiMotorPositionSubsystem m_instance = null;

//     public ExampleMultiMotorPositionSubsystem(MultiMotorPositionSubsystemConstants constants) {
//         super(constants);
//     }

//     public static ExampleMultiMotorPositionSubsystem getInstance() {
//         if (m_instance == null) {
//             m_instance = new ExampleMultiMotorPositionSubsystem(ExampleConstants.ExampleMultiMotorPositionSubsystemConstants);
//         }

//         return m_instance;
//     }

//     @Override
//     public void manualControl() {}

//     @Override
//     public void subsystemPeriodic() {}

//     @Override
//     public void outputTelemetry() {}
    

//     public enum ExampleMultiMotorPositionState implements MultiMotorPositionSubsystemState {
//         DOWN(new double[] {0, 0}, new double[] {0, 0}, "Down"),
//         UP(new double[] {45, 45}, new double[] {0, 0}, "Up"),
//         TRANSITION(new double[] {0, 0}, new double[] {0, 0}, "Transition"),
//         MANUAL(new double[] {0, 0}, new double[] {0, 0}, "Manual");
    
//         private double[] position;
//         private double[] velocity;
//         private String name;
    
//         private ExampleMultiMotorPositionState(double[] position, double[] velocity, String name) {
//           this.position = position;
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
//         public void setVelocity(double velocity, int index) {
//             this.velocity[index] = velocity;
//         }

//         @Override
//         public double[] getPosition() {
//             return position;
//         }

//         @Override
//         public void setPosition(double position, int index) {
//             this.position[index] = position;
//         }
//     }
// }