// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Swerve extends SubsystemBase {

  final CANSparkFlex wheelMotor;
  final CANSparkMax swivelMotor;

  /** Creates a new Intake. */
  public Swerve() {
    wheelMotor = new CANSparkFlex(
	    Constants.wheelMotorId, MotorType.kBrushless);
    swivelMotor = new CANSparkMax(
      Constants.swivelMotorId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void go(double power){
    wheelMotor.set(power);
    swivelMotor.set(power);
  }
}
