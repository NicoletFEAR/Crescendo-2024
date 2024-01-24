// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  public static CommandXboxController m_driverController = new CommandXboxController(0);
  
  // public static Launcher m_launcher = new Launcher();
  // public static Wrist m_wrist = new Wrist();

  public static Pigeon2 m_pigeon = new Pigeon2(0, "rio");

  public RobotContainer() {
    m_pigeon.setYaw(0);
    configureBindings();
  }

  private void configureBindings() {
    // m_driverController.a().whileTrue(new RunLaunchMotors(m_launcher, 3, 1.5));
    // m_driverController.b().whileTrue(new RunLaunchMotors(m_launcher, 6, 0));
    // m_driverController.x().whileTrue(new RunLaunchMotors(m_launcher, 9, 1.5));
    // m_driverController.y().whileTrue(new RunLaunchMotors(m_launcher, 12, 6));
    // m_driverController.leftBumper().whileTrue(new RunLaunchMotors(m_launcher, -3, -3));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
