// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  final CANSparkMax intakeRollerMotor;
  final CANSparkMax intakeWristMotor;
  final RelativeEncoder intakeWristEndcoder;

  /** Creates a new Intake. */
  public Intake() {
    intakeRollerMotor = new CANSparkMax(Constants.intakeRollerMotorID, MotorType.kBrushless);
    intakeWristMotor = new CANSparkMax(Constants.intakeWristMotorID, MotorType.kBrushless);
    intakeWristEndcoder = intakeWristMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveRoller(double speed){
    intakeRollerMotor.set(speed);
  }

  public void driveWrist(double speed){
    intakeWristMotor.set(speed);
  }

  public void setWristZero(){
    intakeWristEndcoder.setPosition(0);
  }
}
