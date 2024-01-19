// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ClimbManualControl;
import frc.robot.Subsystems.Climb;

/** Add your docs here. */
public class RobotContainer {
    private CommandXboxController m_copilotController; 
    public final Climb m_Climb = new Climb();

    public RobotContainer(){
        m_Climb.setDefaultCommand(new ClimbManualControl(m_Climb, m_copilotController));
        configureButtonBindings();
    }
    
    public CommandXboxController getCopilotXboxController(){
        return m_copilotController;
    }

    private void configureButtonBindings(){
        m_copilotController = new CommandXboxController(1);
        m_copilotController.a().onTrue(new ClimbSetpoint(m_Climb));
        m_copilotController.y().onTrue(new ClimbTuckedInSetpoint(m_Climb));
    }
}
