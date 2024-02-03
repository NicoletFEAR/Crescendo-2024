// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ControlMotor;
import frc.robot.commands.SetMotorSpeed;
import frc.robot.subsystems.MotorFEAR;

public class RobotContainer {

  public static CommandXboxController m_driverController = new CommandXboxController(0);
  public static MotorFEAR m_flex;

  public RobotContainer() {
    m_flex = new MotorFEAR();

    configureBindings();
  }

  private void configureBindings() {
    m_flex.setDefaultCommand(new ControlMotor(m_flex));
    m_driverController.b().onTrue(new SetMotorSpeed(m_flex, 1));
    m_driverController.x().onTrue(new SetMotorSpeed(m_flex, -1));
    m_driverController.a().onTrue(new SetMotorSpeed(m_flex, 0));
  }

  public static CommandXboxController getController(){
    return m_driverController;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
