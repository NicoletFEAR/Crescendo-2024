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
private final CANSparkMax leftWinchMotor;
private final CANSparkMax rightWinchMotor;

private final RelativeEncoder leftWinchEncoder;
private final RelativeEncoder rightWinchEncoder;
  
private double kp = 0.15;
private double ki = 0.0;
private double kd = 0.0;

private double maxClimbRotations = 10.0;
private double minClimbRotations = 0.0;
private double climbDeadzone = 0.5;
private double climbSetpoint = 7.7;
    
private PIDController climbPID;

RobotContainer robotContainer;  

  /** Creates a new Climb. */
  public Climb(RobotContainer newRobotContainer) {
    leftWinchMotor = new CANSparkMax(Constants.leftWristMotorID, MotorType.kBrushless);
    rightWinchMotor = new CANSparkMax(Constants.leftWristMotorID, MotorType.kBrushless);
    rightWinchMotor.follow(leftWinchMotor, true);

    leftWinchEncoder = leftWinchMotor.getEncoder();
    rightWinchEncoder = rightWinchMotor.getEncoder();
    leftWinchEncoder.setPosition(0);
    rightWinchEncoder.setPosition(0);

    climbPID = new PIDController(kp, ki, kd);
    climbPID.setTolerance(climbDeadzone);
    setIntendedPosition(0);

    SmartDashboard.putNumber("kp", kp);
    SmartDashboard.putNumber("ki", ki);
    SmartDashboard.putNumber("kd", kd);
    
    SmartDashboard.putNumber("min rotations", minClimbRotations);
    SmartDashboard.putNumber("max rotations", maxClimbRotations);
    SmartDashboard.putNumber("deadzone", climbDeadzone);
    SmartDashboard.putNumber("climb setpoint", climbSetPoint);

    SmartDashboard.putNumber("inteded rotations", getIntendedPosition());
    SmartDashboard.putNumber("current rotations", getCurrentPosition());
    
    SmartDashboard.putNumber("motor speed", climbPID.calculate(getCurrentPosition()));

    SmartDashboard.putboolean("at position", atPosiion());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("inteded rotations", getIntendedPosition());
    SmartDashboard.putNumber("current rotations", getCurrentPosition());
    
    SmartDashboard.putNumber("motor speed", climbPID.calculate(getCurrentPosition());

    climbPID.setP(SmartDashboard.getNumber("kp", kp));
    climbPID.setI(SmartDashboard.getNumber("ki", ki));
    climbPID.setD(SmartDashboard.getNumber("kd", kd));
    climbPID.setTolerance(SmartDashboard.getNumber("deadzone", climbDeadzone));

    // when limit switch is hit call zeroCurrentPosition() and zeroIntendedPosition()

  }

  public void setIntendedPosition(double newPosition) {
    if(newPosition < maxClimbRotations && newPosition > minClimbRotations){
      climbPID.setSetpoint(newPosition);
    }
  }

  public double getIntendedPosition(){
    return climbPID.getSetpoint();
  }

  public double getCurrentPosition(){
    leftWinchEncoder.getPosition();
  }

  public void zeroCurrentPosition(){
    currentPosition = 0.0;
  }

  public void zeroIntendedPosition(){
    setIntendedPosition(0);
  }

  public double getClimbDeadzone(){
    return climbDeadzone;
  }

  public double getClimbSetpoint(){
    return climbSetpoint;
  }

  public void manualControl(value){
    double deltaPosition = value;
    
    if(Math.abs(deltaPosition) < 0.15){
      deltaPosition = 0.0;
      // changing small inputs to zero stop stick drift from moving the climb mech
    }

    if (currentPosition < maxClimbRotations && currentPosition > minClimbRotations){
      setIntendedPosition(getIntendedPosition() + deltaPosition);
      // changes our intended position, smoothly moving the arm as long as it's within bounds
    }
    else if (currentPosition < minClimbRotations) {
      setIntendedPosition(minClimbRotations);
      // moves the arm up if it is too low
      
      if(delta position > 0){
        setIntendedPosition(getIntendedPosition() + deltaPosition);
        // lets the copilot raise the arm if the arm is too low
      }
    }
    else if (currentPosition > maxClimbRotations) {
      setIntendedPosition(minCLimbRotations);
      // moves the arm down if it is too high
      
      if (deltaPosition < 0){
        setIntendedPosition(getIntendedPosition() + deltaPosition);
        // lets the copilot lower the arm if the arm is too high
      }
    }
  }

  public void drivePID(){
    leftWinchMotor.set(climbPID.calculate(getcurrentPosition()));
  }

  public boolean atPosition(){
    if (Math.abs(currentPosition - getIntendedPosition()) < deadzone){
      return true;
    }
    return false;
  } 

  public void stop(){
    leftWinchMotor.set(0);
  }

}
