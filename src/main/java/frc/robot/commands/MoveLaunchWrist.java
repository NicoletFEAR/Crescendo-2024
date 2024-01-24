// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.Launcher;

// public class MoveLaunchWrist extends Command {
//   Launcher m_Launcher;
//   CommandXboxController m_CommandXboxController;
  
//   /** Creates a new LaunchPitch. */
//   public MoveLaunchWrist(Launcher newLauncher, CommandXboxController newControl) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     m_Launcher = newLauncher;
//     m_CommandXboxController = newControl;
//     addRequirements(m_Launcher);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_Launcher.runMotor(12 * m_CommandXboxController.getLeftY(), 12 * m_CommandXboxController.getLeftY());
//     SmartDashboard.putNumber("launch volt", 12 * m_CommandXboxController.getLeftY());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
