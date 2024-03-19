package frc.lib.templates;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public abstract class SuperstructureSubsystem extends SubsystemBase {

  protected SuperstructureState m_currentState;
  protected SuperstructureState m_desiredState;

  protected String m_name;

  public SuperstructureSubsystem(SuperstructureState initialState, String name) {
    m_currentState = initialState;
    m_desiredState = initialState;
    m_name = name;
  }

  public void setDesiredState(SuperstructureState desiredState) {
    m_desiredState = desiredState;
  }

  public void setCurrentState(SuperstructureState currentState) {
    m_currentState = currentState;
  }

  public SuperstructureState getDesiredState() {
    return m_desiredState;
  }

  public SuperstructureState getCurrentState() {
    return m_currentState;
  }

  public abstract SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState);
  
  public abstract SuperstructureState getTransitionState();

  public abstract void superstructurePeriodic();

  public interface SuperstructureState {
    String getName();
  }

  @Override
  public void periodic() {
    superstructurePeriodic();

    if (Constants.kInfoMode) {
      SmartDashboard.putString(m_name + "/Current State", m_currentState.getName());
      SmartDashboard.putString(m_name + "/Desired State", m_desiredState.getName());
    }
  }
}
