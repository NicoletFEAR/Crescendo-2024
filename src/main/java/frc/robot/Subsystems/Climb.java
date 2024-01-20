// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb extends SubsystemBase {
  private final CANSparkMax leftClimbMotor;
  private final CANSparkMax rightClimbMotor;

  private final RelativeEncoder leftClimbEncoder;
  private final RelativeEncoder rightClimbEncoder;
  
  private double kp = 0.15;
  private double ki = 0.0;
  private double kd = 0.0;

  private double maxClimbPosition = 10.0;
  private double minClimbPosition = 0.0;
  private double climbChainSetpoint = 7.7;
    
  private PIDController climbPID;

  DigitalInput climbLimitSwitch;


  /** Creates a new Climb. */
  public Climb() {
    leftClimbMotor = new CANSparkMax(Constants.leftClimbMotorID, MotorType.kBrushless);
    rightClimbMotor = new CANSparkMax(Constants.rightClimbMotorID, MotorType.kBrushless);
    rightClimbMotor.follow(leftClimbMotor, true);

    leftClimbEncoder = leftClimbMotor.getEncoder();
    rightClimbEncoder = rightClimbMotor.getEncoder();
    leftClimbEncoder.setPosition(0);
    rightClimbEncoder.setPosition(0);

    climbPID = new PIDController(kp, ki, kd);
    climbPID.setTolerance(0.5);
    climbPID.setSetpoint(0);

    climbLimitSwitch = new DigitalInput(0);
    
    SmartDashboard.putNumber("kp", kp);
    SmartDashboard.putNumber("ki", ki);
    SmartDashboard.putNumber("kd", kd);
    
    SmartDashboard.putNumber("min climb position", minClimbPosition);
    SmartDashboard.putNumber("max climb position", maxClimbPosition);
    SmartDashboard.putNumber("climb deadzone", getClimbDeadzone());
    SmartDashboard.putNumber("climb chain setpoint", climbChainSetpoint);

    SmartDashboard.putNumber("inteded climb position", getClimbIntendedPosition());
    SmartDashboard.putNumber("current climb position", getClimbCurrentPosition());
    
    SmartDashboard.putNumber("climb motor speed", climbPID.calculate(getClimbCurrentPosition()));

    SmartDashboard.putBoolean("climb at position", atPosition());
    SmartDashboard.putBoolean("Limit Switch Y/N", getClimbLimitSwitch());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("inteded climb Position", getClimbIntendedPosition());
    SmartDashboard.putNumber("current climb Position", getClimbCurrentPosition());
    SmartDashboard.putNumber("climb motor speed", climbPID.calculate(getClimbCurrentPosition()));

    SmartDashboard.putBoolean("Limit Switch Y/N", getClimbLimitSwitch());


    climbPID.setP(SmartDashboard.getNumber("kp", kp));
    climbPID.setI(SmartDashboard.getNumber("ki", ki));
    climbPID.setD(SmartDashboard.getNumber("kd", kd));
    climbPID.setTolerance(SmartDashboard.getNumber("climb deadzone", getClimbDeadzone()));

    // when limit switch is hit call zeroClimbCurrentPosition() and zeroClimbIntendedPosition()

    driveClimb();
  }

  public void setClimbIntendedPosition(double newPosition) {
    if(newPosition < maxClimbPosition && newPosition > minClimbPosition){
      climbPID.setSetpoint(newPosition);
    }
  }

  public boolean getClimbLimitSwitch() {
    return !climbLimitSwitch.get();

  }

  public double getClimbIntendedPosition(){
    return climbPID.getSetpoint();
  }

  public double getClimbCurrentPosition(){
    return leftClimbEncoder.getPosition();
  }

  public void zeroClimbCurrentPosition(){
    leftClimbEncoder.setPosition(0.0);
  }

  public void zeroClimbIntendedPosition(){
    setClimbIntendedPosition(0);
  }

  public double getClimbChainSetpoint(){
    return climbChainSetpoint;
  }

  public void climbManualControl(double value){
    double deltaPosition = value;
    
    if(Math.abs(deltaPosition) < 0.15){
      deltaPosition = 0.0;
      // changing small inputs to zero stop stick drift from moving the climb mech
    }

    if (getClimbCurrentPosition() < maxClimbPosition && getClimbCurrentPosition() > minClimbPosition){
      setClimbIntendedPosition(getClimbIntendedPosition() + deltaPosition);
      // changes our intended position, smoothly moving the arm as long as it's within bounds
    }
    else if (getClimbCurrentPosition() < minClimbPosition) {
      setClimbIntendedPosition(minClimbPosition);
      // moves the arm up if it is too low
      
      if(deltaPosition > 0){
        setClimbIntendedPosition(getClimbIntendedPosition() + deltaPosition);
        // lets the copilot raise the arm if the arm is too low
      }
    }
    else if (getClimbCurrentPosition() > maxClimbPosition) {
      setClimbIntendedPosition(minClimbPosition);
      // moves the arm down if it is too high
      
      if (deltaPosition < 0){
        setClimbIntendedPosition(getClimbIntendedPosition() + deltaPosition);
        // lets the copilot lower the arm if the arm is too high
      }
    }
  }

  public void driveClimb(){
    leftClimbMotor.set(climbPID.calculate(getClimbCurrentPosition()));
  }

  public boolean atPosition(){
    if (Math.abs(getClimbCurrentPosition() - getClimbIntendedPosition()) < getClimbDeadzone()){
      return true;
    }
    return false;
  } 

  public double getClimbDeadzone(){
    return climbPID.getPositionTolerance();

  }

  public void stop(){
    leftClimbMotor.set(0);
  }

}
