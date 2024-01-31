// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeFlywheel.IntakeFlywheelState;
import frc.robot.subsystems.intake.IntakeFlywheel;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure.IntakeSuperstructureState;

public class IntakeFlywheelBeamBreakStop extends Command {
  IntakeFlywheel m_intakeFlywheel;
  IntakeSuperstructure m_intakeSuperstructure;

  public IntakeFlywheelBeamBreakStop(IntakeSuperstructure newIntakeSuperstructure) {
    m_intakeSuperstructure = newIntakeSuperstructure;
    m_intakeFlywheel = m_intakeSuperstructure.getIntakeFlywheel();

    addRequirements(m_intakeSuperstructure);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_intakeFlywheel.getCurrentState() == IntakeFlywheelState.IN && m_intakeFlywheel.getBeamBreak()){
      m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.TRANSITION);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
