// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CheckBeamBreak;
import frc.robot.commands.DriveIntake;
import frc.robot.commands.SetRollerSpeed;
import frc.robot.subsystems.Intake;

//----------------------------------------------//

public class RobotContainer{
  public static CommandXboxController m_driverController = new CommandXboxController(0);

  private final Intake m_intake = new Intake();

  public RobotContainer() {
    m_intake.setDefaultCommand(new DriveIntake(m_intake, m_driverController.getRightY()));
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.y().onTrue(new SetRollerSpeed(m_intake, 0.2)); // represents when the copilot is launching
    m_driverController.y().onFalse(new SetRollerSpeed(m_intake, 0.0));
    m_driverController.a().onTrue(new SetRollerSpeed(m_intake, 0.2)); // represents when the copilot is intaking
    m_driverController.a().onFalse(new SetRollerSpeed(m_intake, 0.0));
    m_driverController.a().onTrue(new CheckBeamBreak(m_intake));
    m_driverController.b().onTrue(new SetRollerSpeed(m_intake, -0.2)); // represents when the copilot is outtaking
    m_driverController.b().onFalse(new SetRollerSpeed(m_intake, 0.0));

    
    
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public double appyDeadband(double value){
    if(Math.abs(value) > 0.15){
      return (value - 0.15) * (1 / (1 - 0.15));
    }
    else{
      return 0;
    }
  }
}