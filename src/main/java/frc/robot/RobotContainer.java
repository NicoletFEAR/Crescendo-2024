// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private CANSparkMax motor1 = new CANSparkMax(34, MotorType.kBrushless);
  private CANSparkMax motor2 = new CANSparkMax(30, MotorType.kBrushless);

  double voltage = 0;

  public RobotContainer() {

    SmartDashboard.putNumber("Motor Voltage", 0);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void periodic() {
    voltage = SmartDashboard.getNumber("Motor Voltage", 0);

    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
  }
}
