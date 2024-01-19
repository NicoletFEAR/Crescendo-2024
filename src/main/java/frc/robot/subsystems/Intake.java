//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  //this iteration of intake code assumes the intake will be where we hold the note and we don't have a dedicated hold
  
  private boolean noteInIntake; // true if there is a note anywhere in the intake
  private boolean noteHeld; // true if the note is securely held is the back of the intake

  private double wristFloorSetpoint = 5;
  private double wristTrapSetpoint = 10;
  private double wristAmpSetpoint = 15;
  
  private double minWristRotation = 0;
  private double maxWristRotation = 20;

  private double wristDeadzone = 0.5;

  double kp = 0.1;
  double ki = 0;
  double kd = 0;

  private final CANSparkMax wristMotor;
  private final CANSparkMax rollerMotor;

  private final RelativeEncoder wristEncoder;

  private final PIDController wristPID;

  /** Creates a new Intake. */
  public Intake() {

    noteHeld = true;
    noteInIntake = true;

    wristMotor = new CANSparkMax(Constants.wristMotorId, MotorType.kBrushless);
    rollerMotor = new CANSparkMax(Constants.rollerMotorId, MotorType.kBrushless);

    wristEncoder = wristMotor.getEncoder();
    wristEncoder.setPosition(0);

    wristPID = new PIDController(kp, ki, kd);
    wristPID.setSetpoint(0);
    wristPID.setTolerance(wristDeadzone);

    SmartDashboard.putNumber("kp", wristPID.getP());
    SmartDashboard.putNumber("ki", wristPID.getI());
    SmartDashboard.putNumber("kd", wristPID.getD());
    
    SmartDashboard.putNumber("current wrist position", wristMotor.getPosition());
    SmartDashboard.putNumber("intended wrist position", wristPID.getSetpoint());
    SmartDashboard.putNumber("wrist motor speed", wristPID.calculate(wristMotor.getPosition()));
    
    SmartDashboard.putNumber("wrist floor intake setpoint", wristFloorSetpoint);
    SmartDashboard.putNumber("wrist trap setpoint", wristTrapSetpoint);
    SmartDashboard.putNumber("wrist amp setpoint", wristAmpSetpoint);
    
    SmartDashboard.putNumber("min wrist rotation", minWristRotation);
    SmartDashboard.putNumber("max wrist rotation", maxWristRotation);
    SmartDashboard.putNumber("wrist deadzone", wristDeadzone);

  }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run

    driveWrist();

    wristPID.setP(SmartDashboard.getNumber("kp", wristPID.getP()));
    wristPID.setI(SmartDashboard.getNumber("ki", wristPID.getI()));
    wristPID.setD(SmartDashboard.getNumber("kd", wristPID.getD()));
    wristPID.setTolerance(SmartDashboard.getNumber("wrist deadzone", wristDeadzone));

    wristFloorSetpoint = SmartDashboard.getNumber("wrist floor intake setpoint", wristFloorSetpoint);
    wristTrapSetpoint = SmartDashboard.getNumber("wrist trap setpoint", wristTrapSetpoint);
    wristAmpSetpoint = SmartDashboard.getNumber("wrist amp setpoint", wristAmpSetpoint);

    periodicNoteStuff(true);
  }

  public void periodicNoteStuff(boolean bool){
    //boolean is just to prevent dead code issues

    if(!noteInIntake){
      if(checkFrontBeamBreak()){
        noteInIntake = true;
      }
    }
    else{
      if(!noteHeld){
        if(checkBackBeamBreak()){
          noteHeld = true;
        }
      }
    }
  }

  public void intake() {
    rollerMotor.set(0.5);
  }

  public void stopWrist() {
    rollerMotor.set(0);
  }

  public void moveNoteToLaunch() {
    rollerMotor.set(0.5);
    noteInIntake = false;
    noteHeld = false;
  }

  public boolean checkFrontBeamBreak(){
    //if front beam break is set off return true
    return true;
  }

  public boolean checkBackBeamBreak(){
   // if back beam break is set off return true
    return true;
  }

  public double getWristFloorSetpoint(){
    return wristFloorSetpoint;
  }

  public double getWristTrapSetpoint(){
    return wristTrapSetpoint;
  }
  
  public double getWristAmpSetpoint(){
    return wristAmpSetpoint;
  }

  public void outtake() {
    rollerMotor.set(-0.5);
    noteHeld = false;
    noteInIntake = false
  }

  public void setIntendedWristPosition(double num) {
    if(num < maxWristRotation && num > minWristRotation){
      wristPID.setSetpoint(num);
  }

  public void driveWrist(){
    wristMotor.set(pid.calculate(wristEncoder.getPosition()));
  }

  public void wristManualControl(value){
    double deltaPosition = value;
    
    if(Math.abs(deltaPosition) < 0.15){
      deltaPosition = 0.0;
      // changing small inputs to zero stop stick drift from moving the Wrist mech
    }

    if (getCurrentWristPosition() < maxWristRotation && getCurrentWristPosition() > minWristRotation){
      setIntendedWristPosition(getIntendedWristPosition() + deltaPosition);
      // changes our intended position, smoothly moving the arm as long as it's within bounds
    }
    else if (getCurrentWristPosition() < minWristRotation) {
      setIntendedWristPosition(minWristRotation);
      // moves the arm up if it is too low
      
      if(delta position > 0){
        setIntendedWristPosition(getIntendedWristPosition() + deltaPosition);
        // lets the copilot raise the arm if the arm is too low
      }
    }
    else if (getCurrentWristPosition() > maxWristRotations) {
      setIntendedWristPosition(minWristRotations);
      // moves the arm down if it is too high
      
      if (deltaPosition < 0){
        setIntendedWristPosition(getIntendedWristPosition() + deltaPosition);
        // lets the copilot lower the arm if the arm is too high
      }
    }
  }

  public double getIntendedWristPosition(){
    return wristPID.getSetPoint();
  }
    
  public double getCurrentWristPosition(){}
    return wristMotor.getPosition();
  }

  public boolean isNoteheld(){
    return noteHeld;
  }
