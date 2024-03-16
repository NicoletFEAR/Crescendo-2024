// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoDrive extends Command {
    private SwerveDrive m_drivebase;
    private double m_translationAxis;
    private double m_strafeAxis;
    private double m_rotationAxis;

  /** Creates a new AutoDrive. */
  public AutoDrive(SwerveDrive drivebase, double translation, double strafe, double rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivebase = drivebase;
    m_translationAxis = translation;
    m_strafeAxis = strafe;
    m_rotationAxis = rotation;

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.drive(m_translationAxis, m_strafeAxis, m_rotationAxis, true, true);
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
