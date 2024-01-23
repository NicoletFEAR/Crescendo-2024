// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private CANSparkMax launchPitchMotor;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder launchPitchEncoder;

  private SparkPIDController leftController;
  private SparkPIDController rightController;

  private CANcoder encoder = new CANcoder(0);

  private double leftDesiredVelocity;
  private double rightDesiredVelocity;

  /** Creates a new launcher. */
  public Launcher() {
    leftMotor = new CANSparkMax(13, MotorType.kBrushless);
    rightMotor = new CANSparkMax(15, MotorType.kBrushless);
    launchPitchMotor = new CANSparkMax(16, MotorType.kBrushless);

    rightMotor.setInverted(true);
    rightMotor.setIdleMode(IdleMode.kCoast);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();
    launchPitchEncoder = launchPitchMotor.getEncoder();

    leftController = leftMotor.getPIDController();
    rightController = rightMotor.getPIDController();

    leftController.setP(LauncherConstants.kp);
    leftController.setI(LauncherConstants.ki);
    leftController.setD(LauncherConstants.kd);

    rightController.setP(LauncherConstants.kp);
    rightController.setI(LauncherConstants.ki);
    rightController.setD(LauncherConstants.kd);


    // encoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor())

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Velocity", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Left Desired Velocity", leftDesiredVelocity);
    SmartDashboard.putNumber("Right Desired Velocity", rightDesiredVelocity);

    SmartDashboard.putNumber("Absolute Position", encoder.getPosition().getValue() * 360);

    
  }

  public void runMotor(double leftVoltage, double rightVoltage) {
    leftMotor.setVoltage(leftVoltage);
    rightMotor.setVoltage(rightVoltage);
  }

  public void incrementIntdended(double value){
    launchPitchMotor.set(value);
    SmartDashboard.putNumber("drive pitch val", value);
  }

  public void setVelocity(double leftVelocity, double rightVelocity) {
    leftController.setReference(leftVelocity, ControlType.kVelocity);
    rightController.setReference(rightVelocity, ControlType.kVelocity);
    leftDesiredVelocity = leftVelocity;
    rightDesiredVelocity = rightVelocity;
  }

  public void stopMotors() {
    leftMotor.setVoltage(0);
    rightMotor.setVoltage(0);
  }

}
