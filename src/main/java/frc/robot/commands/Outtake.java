// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Outtake extends CommandBase {
  // assumes the intake is in proper position
  // should be running while button is held
  
  public Intake m_Intake;

  public SetWristGoal(Intake newIntake) {
    m_Intake = newIntake;

    addRequirements(newIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.outtake(){
  }
  
   // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // update
  }
}
