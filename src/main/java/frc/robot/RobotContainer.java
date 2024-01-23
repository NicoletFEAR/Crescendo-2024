// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ManualWrist;
import frc.robot.commands.RunLaunchMotors;
import frc.robot.commands.SetLaunchVelocity;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Wrist;

public class RobotContainer {

  public static CommandXboxController m_driverController = new CommandXboxController(0);
  
  public static Launcher m_launcher = new Launcher();
  public static Wrist m_wrist = new Wrist();

  public static Pigeon2 m_pigeon = new Pigeon2(1, "rio");

  public RobotContainer() {
    m_pigeon.setYaw(0);

    m_wrist.setDefaultCommand(new ManualWrist(m_wrist));
    configureBindings();
  }

  private void configureBindings() {
    // m_driverController.a().whileTrue(new RunLaunchMotors(m_launcher, -3, -3));
    // m_driverController.b().whileTrue(new RunLaunchMotors(m_launcher, -6, -6));
    // m_driverController.x().whileTrue(new RunLaunchMotors(m_launcher, -9, -9));
    // m_driverController.y().whileTrue(new RunLaunchMotors(m_launcher, -12, -12));
    m_driverController.a().whileTrue(new SetLaunchVelocity(m_launcher, -5000, -5000));
    m_driverController.b().whileTrue(new SetLaunchVelocity(m_launcher, -2500, -2500));
    m_driverController.x().whileTrue(new SetLaunchVelocity(m_launcher, -4000, -4000));
    m_driverController.y().whileTrue(new SetLaunchVelocity(m_launcher, -100, -100));
    m_driverController.leftBumper().whileTrue(new RunLaunchMotors(m_launcher, 3, 3));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
