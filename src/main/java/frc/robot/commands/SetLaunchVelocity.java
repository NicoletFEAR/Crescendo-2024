// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Launcher;

// public class SetLaunchVelocity extends Command {

//   private double m_leftVelocity;
//   private double m_rightVelocity;
//   private Launcher m_launcher; 
//   /** Creates a new RunLaunchMotors. */
//   public SetLaunchVelocity(Launcher launcher, double leftVelocity, double rightVelocity) {
//     m_leftVelocity = leftVelocity;
//     m_rightVelocity = rightVelocity;
//     m_launcher = launcher;
//     addRequirements(launcher);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_launcher.setVelocity(m_leftVelocity, m_rightVelocity);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_launcher.setVelocity(0, 0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
