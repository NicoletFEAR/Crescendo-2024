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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  final CANSparkMax intakeRollerMotor;
  final CANSparkMax intakeWristMotor;
  final RelativeEncoder intakeWristEndcoder;
  final PIDController intakeWristPID;
  final double intakeWristMaxSetpoint = 10;
  final double intakeWristMinSetpoint = 0;
  final double intakeWristFloorSetpoint = 1;
  final double intakeWristAmpSetpoint = 3;
  final double intakeWristInSetpoint = 8;  
  double intakeWristVariableSetpoint = 5;
  final DigitalInput frontBeamBreak;
  final DigitalInput backBeamBreak;

  /** Creates a new Intake. */
  public Intake() {
    intakeRollerMotor = new CANSparkMax(Constants.intakeRollerMotorID, MotorType.kBrushless);
    intakeWristMotor = new CANSparkMax(Constants.intakeWristMotorID, MotorType.kBrushless);
    intakeWristEndcoder = intakeWristMotor.getEncoder();
    
    intakeWristPID = new PIDController(0.1, 0, 0);
    intakeWristPID.setTolerance(0.5);
    SmartDashboard.putNumber("Intake Wrist Kp", intakeWristPID.getP());
    SmartDashboard.putNumber("Intake Wrist Ki", intakeWristPID.getI());
    SmartDashboard.putNumber("Intake Wrist Kd", intakeWristPID.getD());
    SmartDashboard.putNumber("Intake Wrist Variable Set Point", intakeWristVariableSetpoint);

    frontBeamBreak = new DigitalInput(0);
    backBeamBreak = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    intakeWristPID.setP(SmartDashboard.getNumber("Intake Wrist Kp", PIDController.getP()));
    intakeWristPID.setI(SmartDashboard.getNumber("Intake Wrist Ki", PIDController.getI()));
    intakeWristPID.setD(SmartDashboard.getNumber("Intake Wrist Kd", PIDController.getD()));
    intakeWristVariableSetpoint = SmartDashboard.getNumber("Intake Wrist Variable Set Point", intakeWristVariableSetpoint);
    SmartDashboard.putBoolean("Front Beam Break", frontBeamBreak.get());
    SmartDashboard.putBoolean("Back Beam Break", backBeamBreak.get());
  }

  public void driveRoller(double speed){
    intakeRollerMotor.set(speed);
  }

  public void driveWrist(double speed){
    intakeWristMotor.set(speed);
  }

  public boolean getFrontBeamBreak(){
    return frontBeamBreak.get();
  }

  public boolean getBackBeamBreak(){
    return backBeamBreak.get();
  }

  public void setWristSetpoint(double position){
    intakeWristPID.setSetpoint(position);
  }

  public double getWristSetpoint(){
    return intakeWristPID.getSetpoint();
  }

  public double getWristPosition(){
    return intakeWristEncoder.getPosition();
  }

  public void setWristZero(){
    intakeWristEndcoder.setPosition(0);
  }
}
