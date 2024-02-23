// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auto;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.util.Units;
// // import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.drivebase.TurnToAngle;
// import frc.robot.subsystems.swerve.SwerveDrive;

// public class CenterNoteAuto extends Command {
//   /** Creates a new CenterNoteAuto. */
//   private boolean[] m_availableNotes;
//   private Command m_noteCommandRunning;
//   private Command m_scoreCommandRunning;
//   private Command m_sequentialCommandRunning;

//   private int lastIndex = 0;

//   PathPlannerPath note5 = PathPlannerPath.fromPathFile("Note 5");
//   PathPlannerPath note4 = PathPlannerPath.fromPathFile("Note 4");
//   PathPlannerPath note3 = PathPlannerPath.fromPathFile("Note 3");
//   PathPlannerPath note2 = PathPlannerPath.fromPathFile("Note 2");
//   PathPlannerPath note1 = PathPlannerPath.fromPathFile("Note 1");

//   private PathPlannerPath[] m_paths = new PathPlannerPath[] {note1, note2, note3, note4, note5};

//   private Pose2d m_scoreTop = new Pose2d(4.67, 6.62, Rotation2d.fromDegrees(-175));
//   private Pose2d m_scoreBot = new Pose2d(4.67, 1.44, Rotation2d.fromDegrees(-210));
//   private Pose2d m_poseToUse = m_scoreBot;

//   private PathConstraints m_constraints = new PathConstraints(3.0, 3.0,
//     Units.degreesToRadians(540), Units.degreesToRadians(720));


//   public CenterNoteAuto() {}

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_availableNotes = new boolean[] {true, true, true, true, true};
//     // m_availableNotes[1] = SmartDashboard.putBoolean("Note 2", true);

//     lastIndex = getIndexToUse();

//     if (lastIndex != -1) {
//       m_poseToUse = lastIndex > 2 ? m_scoreBot : m_scoreTop;

//       m_noteCommandRunning = AutoBuilder.pathfindThenFollowPath(m_paths[lastIndex], m_constraints);
//       m_scoreCommandRunning = AutoBuilder.pathfindToPose(m_poseToUse, m_constraints);
//       m_sequentialCommandRunning = new SequentialCommandGroup(m_noteCommandRunning, m_scoreCommandRunning, new TurnToAngle(SwerveDrive.getInstance()), new InstantCommand(() -> m_availableNotes[lastIndex] = false));
//       m_sequentialCommandRunning.schedule();
//     }

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     // m_availableNotes[1] = SmartDashboard.getBoolean("Note 2", true);
//     // SmartDashboard.putBooleanArray("Notes", m_availableNotes);
    
//     if (m_noteCommandRunning != null
//      || m_availableNotes[lastIndex] == false
//      ) {
//       if (!CommandScheduler.getInstance().isScheduled(m_sequentialCommandRunning)
//        || m_availableNotes[lastIndex] == false
//        ) {
//         lastIndex = getIndexToUse();
//         if (lastIndex != -1) {
//           m_poseToUse = lastIndex > 2 ? m_scoreBot : m_scoreTop;
//           m_noteCommandRunning = AutoBuilder.pathfindThenFollowPath(m_paths[lastIndex], m_constraints);
//           m_scoreCommandRunning = AutoBuilder.pathfindToPose(m_poseToUse, m_constraints);
//           m_sequentialCommandRunning = new SequentialCommandGroup(m_noteCommandRunning, m_scoreCommandRunning, new TurnToAngle(SwerveDrive.getInstance()), new InstantCommand(() -> m_availableNotes[lastIndex] = false));
//           m_sequentialCommandRunning.schedule();
//         }
//       }
//     }
//   }


//   private int getIndexToUse() {
//     double minDist = -1;
//     int indexToUse = -1;
//     for (int i = 0; i < m_availableNotes.length; i++) {
//       if (m_availableNotes[i]) {
//         double dist = SwerveDrive.getInstance().getPose().getTranslation().getDistance(m_paths[i].getPreviewStartingHolonomicPose().getTranslation());
//         if (minDist == -1 || dist < minDist) {
//           minDist = dist;
//           indexToUse = i;
//         }
//       }
//     }
//     return indexToUse;
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return getIndexToUse() == -1;
//   }
// }
