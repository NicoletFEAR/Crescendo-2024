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

  boolean noteInIntake;
  boolean noteHeld;

  double intakeRotation;
  double intakeGoalRotation;

  public final double ampScoreRotation = 5;
  public final double trapScoreRotation = 10;
  public final double floorIntakeRotation = 15;
  public final double minRotation = 0;
  public final double maxRotation = 20;

  double kp = 0.1;
  double ki = 0;
  double kd = 0;

  CANSparkMax wristMotor;
  CANSparkMax rollerMotor;

  RelativeEncoder wristEncoder;

  PIDController pid;

  /** Creates a new Intake. */
  public Intake() {

    noteHeld = true;
    noteInIntake = true;

    wristMotor = new CANSparkMax(Constants.wristMotorId, MotorType.kBrushless);
    // rollerMotor = new CANSparkMax(Constants.rollerMotorId, MotorType.kBrushless);

    wristEncoder = wristMotor.getEncoder();
    wristEncoder.setPosition(10);


    pid = new PIDController(kp, ki, kd);
    pid.setSetpoint(10);
    pid.setTolerance(0.5);

    SmartDashboard.putNumber("kp", pid.getP());
    SmartDashboard.putNumber("ki", pid.getI());
    SmartDashboard.putNumber("kd", pid.getD());

  }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Pos", wristEncoder.getPosition());
    SmartDashboard.putNumber("Cureent Goal", pid.getSetpoint());
    SmartDashboard.putNumber("Wrist Drive", pid.calculate(wristEncoder.getPosition()));
    
    

    pid.setP(SmartDashboard.getNumber("kp", pid.getP()));
    pid.setI(SmartDashboard.getNumber("ki", pid.getI()));
    pid.setD(SmartDashboard.getNumber("kd", pid.getD()));

    // periodicNoteStuff(true);
  }

  // public void periodicNoteStuff(boolean bool){
  //   // boolean is just to prevent dead code issues

  //   if(! noteInIntake){
  //     if(checkFrontBeamBreak()){
  //       noteInIntake = true;
  //     }
  //   }
  //   else{
  //     if(noteHeld){
  //       if(bool){
  //         // button for launcher is pressed
  //         // launch
  //         launch();
  //         noteHeld = false;
  //         noteInIntake = false;

  //       }
  //       else if(true){
  //         // button for outtake is pressed
  //         // outtake
  //         outtake();
  //         noteHeld = false;

  //         if(checkFrontBeamBreak()){
  //           noteInIntake = false;
  //           // once the note is out the front we dont have it
  //         }
        
  //       }
        
  //     }
  //     else{
  //       if(checkBackBeamBreak()){
  //         noteHeld = true;
  //         stop();
  //       }
  //     }
  //   }
  // }

  // public void intake() {
  //   rollerMotor.set(0.2);
  // }

  // public void stop() {
  //   rollerMotor.set(0);
  // }

  // public void launch() {
  //   rollerMotor.set(0.7);
  // }

  public boolean checkFrontBeamBreak(){
    // if front beam break is set off return true
    return true;
  }

  public boolean checkBackBeamBreak(){
    // if back beam break is set off return true
    return true;
  }

  // public void outtake() {
  //   rollerMotor.set(-0.5);
  // }

  public void setGoalRotation(int num) {
    if(num == 1){
      pid.setSetpoint(ampScoreRotation);
    }
    else if (num == 2){
      pid.setSetpoint(trapScoreRotation);
    }
    else if (num == 3){
      pid.setSetpoint(floorIntakeRotation);
    }
  }

  public void driveWrist(){
    wristMotor.set(pid.calculate(wristEncoder.getPosition()));
  }

}