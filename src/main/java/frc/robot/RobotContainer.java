// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDState;

public class RobotContainer {

  public static CommandXboxController m_driverController = new CommandXboxController(0);

  public static LED m_led = LED.getInstance();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.a().onTrue(new InstantCommand(() -> m_led.setState(LEDState.BLUE)));
    m_driverController.b().onTrue(new InstantCommand(() -> m_led.setState(LEDState.GREEN)));
    m_driverController.x().onTrue(new InstantCommand(() -> m_led.setState(LEDState.TEAL_WIPE)));
    m_driverController.y().onTrue(new InstantCommand(() -> m_led.setState(LEDState.RAINBOW)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
