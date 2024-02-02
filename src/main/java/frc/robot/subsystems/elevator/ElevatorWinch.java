// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.templates.PositionSubsystem;
import frc.robot.subsystems.templates.SubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;

public class ElevatorWinch extends PositionSubsystem {
  private static ElevatorWinch m_instance = null;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorWinch(PositionSubsystemConstants constants) {
    super(constants);
  }
  
  public static ElevatorWinch getInstance() {
        if (m_instance == null) {
            m_instance = new ElevatorWinch(ElevatorConstants.kElevatorWinchConstants);
        }

        return m_instance;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void subsystemPeriodic() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'subsystemPeriodic'");
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
  }
  public enum ElevatorWinchState implements PositionSubsystemState {
    HIGH(0, 0, "Down"),
    LOW(100, 0, "Up"),
    TRANSITION(0, 0, "Transition"),
    MANUAL(0, 0, "Manual");

    private double position;
    private double velocity;
    private String name;

    private ElevatorWinchState(double position, double velocity, String name) {
      this.position = position;
      this.velocity = velocity;
      this.name = name;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public double getVelocity() {
        return velocity;
    }

    @Override
    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public void setPosition(double position) {
        this.position = position;
    }
}

}

