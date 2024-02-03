// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class MotorFEAR extends SubsystemBase {
  CANSparkMax motor = new CANSparkMax(Constants.CANSparkID, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();

  /** Creates a new MotorFEAR. */
  public MotorFEAR() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("flex speed", motor.get());
    SmartDashboard.putNumber("flex pos", encoder.getPosition());
  }

  public void set(double velocity){
    motor.set(velocity);
  }

  public void setControllerSpeed(){
    motor.set(RobotContainer.getController().getLeftY());
  }
}
