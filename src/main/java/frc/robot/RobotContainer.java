// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDState;


public class RobotContainer {

  public static CommandXboxController m_driverController = new CommandXboxController(0);

  public static LEDs m_led;

  public static BeamBreak m_beamBreak;


  public RobotContainer() {
    m_led = LEDs.getInstance();
    m_beamBreak = new BeamBreak();
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.x().onTrue(new InstantCommand(() -> m_led.setState(LEDState.BLUE)));
    m_driverController.a().onTrue(new InstantCommand(() -> m_led.setState(LEDState.GREEN)));
    m_driverController.b().onTrue(new InstantCommand(() -> m_led.setState(LEDState.RED)));
    m_driverController.y().onTrue(new InstantCommand(() -> m_led.setState(LEDState.RAINBOW)));
    // m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_led.setState(LEDState.TEAL_WIPE)));
    // m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_led.setState(LEDState.TEAL_PULSE)));
    // m_driverController.back().onTrue(new InstantCommand(() -> m_led.setState(LEDState.TEAL_RAIN)));
    // m_driverController.pov(0).onTrue(new InstantCommand(() -> m_led.setState(LEDState.BLUE_FLASH)));
    // m_driverController.pov(180).onTrue(new InstantCommand(() -> m_led.setState(LEDState.ORANGE_FLASH)));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
