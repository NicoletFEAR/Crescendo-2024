// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake extends SubsystemBase {

  final CANSparkMax intakeRollerMotor;
  final CANSparkMax intakeRollerSlaveMotor;
  final CANSparkMax intakeWristMotor;
  final RelativeEncoder intakeWristEndcoder;
  final PIDController intakeWristPID;
  
  final double intakeWristMaxSetpoint = 10;
  final double intakeWristMinSetpoint = 0;
  final double intakeWristFloorSetpoint = 1;
  final double intakeWristAmpSetpoint = 3;
  final double intakeWristInSetpoint = 8;  
  double intakeWristVariableSetpoint = 5;
  
  public final DigitalInput backBeamBreak;

  /** Creates a new Intake. */
  public Intake() {
    intakeRollerMotor = new CANSparkMax(17, MotorType.kBrushless);
    intakeRollerSlaveMotor = new CANSparkMax(18, MotorType.kBrushless);
    intakeRollerSlaveMotor.follow(intakeRollerMotor, true);

    intakeWristMotor = new CANSparkMax(Constants.intakeWristMotorID, MotorType.kBrushless);
    intakeWristEndcoder = intakeWristMotor.getEncoder();
    
    intakeWristPID = new PIDController(0.1, 0, 0);
    intakeWristPID.setTolerance(0.5);
    SmartDashboard.putNumber("Intake Wrist Kp", intakeWristPID.getP());
    SmartDashboard.putNumber("Intake Wrist Ki", intakeWristPID.getI());
    SmartDashboard.putNumber("Intake Wrist Kd", intakeWristPID.getD());
    SmartDashboard.putNumber("Intake Wrist Variable Set Point", intakeWristVariableSetpoint);

    backBeamBreak = new DigitalInput(0);
  }

  @Override
  public void periodic() {
    intakeWristPID.setP(SmartDashboard.getNumber("Intake Wrist Kp", intakeWristPID.getP()));
    intakeWristPID.setI(SmartDashboard.getNumber("Intake Wrist Ki", intakeWristPID.getI()));
    intakeWristPID.setD(SmartDashboard.getNumber("Intake Wrist Kd", intakeWristPID.getD()));
    intakeWristVariableSetpoint = SmartDashboard.getNumber("Intake Wrist Variable Set Point", intakeWristVariableSetpoint);
    SmartDashboard.putBoolean("Back Beam Break", backBeamBreak.get());
    
  }

  public void driveRoller(double speed){
    intakeRollerMotor.set(speed);
  }

  public void driveWrist(double speed){
    intakeWristMotor.set(speed);
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
    return intakeWristEndcoder.getPosition();
  }

  public void setWristZero(){
    intakeWristEndcoder.setPosition(0);
  }

  public boolean atPosition(){
    if(Math.abs(getWristSetpoint() - getWristPosition()) < intakeWristPID.getPositionTolerance()){
        return true;
    }

    return false;
  }
  
}
