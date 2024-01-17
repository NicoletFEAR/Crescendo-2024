//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  boolean noteInFrontIntake;
  boolean noteInBackIntake;

  boolean canIntake;
  boolean canLaunch;


  double intakeRotation;
  double intakeGoalRotation;

  public final double ampScoreRotation = 0.5;
  public final double trapScoreRotation = 0.5;
  public final double floorIntakeRotation = 0.5;

  CANSparkMax wristMotor;
  CANSparkMax rollerMotor;

  /** Creates a new Intake. */
  public Intake() {

    noteInFrontIntake = true;
    noteInBackIntake = true;

    intakeRotation = 0;

    wristMotor = new CANSparkMax(Constants.wristMotorId, MotorType.kBrushless);
    rollerMotor = new CANSparkMax(Constants.rollerMotorId, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run
  }

  public void intake() {
    rollerMotor.set(0.5);
  }

  public void checkNote(){

  }

  public void outtake() {
    
    rollerMotor.set(-0.5);

  }

  public void setGoalRotation(double newGoalPosition) {

    intakeGoalRotation = newGoalPosition;

  }



}