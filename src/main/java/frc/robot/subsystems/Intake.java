// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase {

  final CANSparkMax flywheelMotor;
  final CANSparkFlex otherMotor;

  /** Creates a new Intake. */
  public Intake() {
    otherMotor = new CANSparkFlex(
      17, MotorType.kBrushless);
    flywheelMotor = new CANSparkMax(
	    Constants.flywheelMotorId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void go(double power){
    flywheelMotor.set(power);
    otherMotor.set(power);
  }
}
