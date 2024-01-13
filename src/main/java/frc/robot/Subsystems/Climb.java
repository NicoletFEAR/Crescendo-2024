// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Climb extends SubsystemBase {
  public static  CANSparkMax leftWinchMotor;
  public static  CANSparkMax rightWinchMotor;

  public static RelativeEncoder leftWinchEncoder;
  //public static RelativeEncoder rightWinchEncoder;
  
  final double kp = 0.01;
  final double ki = 0.0;
  final double kd = 0.0; 
  double currentPosition = 0.0;
  double maxRotations = 10.0;
  double minRotations = 0.0;
  double deadzone = 0.5;
    
  PIDController climbPID;
  RobotContainer robotContainer;

  final double maxVoltage = 2;
  


  /** Creates a new Climb. */
  public Climb(RobotContainer newRobotContainer) {
    leftWinchMotor = new CANSparkMax(14, MotorType.kBrushless);
    //rightWinchMotor = new CANSparkMax(0, MotorType.kBrushless);

    leftWinchEncoder = leftWinchMotor.getEncoder();
    //rightWinchEncoder = rightWinchMotor.getEncoder();

    climbPID = new PIDController(kp, ki, kd);
    climbPID.setTolerance(deadzone);
    setIntendedPosition(0.0);

    leftWinchEncoder.setPosition(0.0);
    //rightWinchEncoder.setPosition(0.0);

    //rightWinchMotor.follow(leftWinchMotor, true);
    
    robotContainer = newRobotContainer;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drive();
    SmartDashboard.putNumber("Current Position", currentPosition);
    SmartDashboard.putNumber("Intended Position", getIntendedPosition());
    SmartDashboard.putNumber("Voltage", 2 * climbPID.calculate(currentPosition));
  }

  public void setIntendedPosition(double newPosition) {
    climbPID.setSetpoint(newPosition);
  }

  public void updateCurrentPosition(){
    currentPosition = leftWinchEncoder.getPosition();
  }

  public double getIntendedPosition(){
    return climbPID.getSetpoint();
  }

  public void zeroCurrentPosition(){
    currentPosition = 0.0;
  }

  public void manualControl(){
    double deltaPosition = robotContainer.getCopilotXboxController().getLeftY();

    if (currentPosition < maxRotations - deadzone && currentPosition > minRotations + deadzone){
      setIntendedPosition(getIntendedPosition() + deltaPosition);
    }
    else if (currentPosition < minRotations) {
      if (deltaPosition > 0){
         setIntendedPosition(getIntendedPosition() + deltaPosition);
      }
    }
    else if (currentPosition > maxRotations) {
      if (deltaPosition < 0){
        setIntendedPosition(getIntendedPosition() + deltaPosition);
      }
    }
  }
  
  public void drive(){
    double voltage = maxVoltage * climbPID.calculate(currentPosition);
    leftWinchMotor.setVoltage(voltage);
  }

  public boolean atPosition(){
    if (Math.abs(currentPosition - getIntendedPosition()) < deadzone){
      return true;
    }
    return false;
  } 
  public void stop(){
    leftWinchMotor.setVoltage(0);
  }

}
