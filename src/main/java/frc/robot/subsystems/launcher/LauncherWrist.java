// package frc.robot.subsystems.launcher;


// import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;

// import com.ctre.phoenix6.hardware.CANcoder;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants.LauncherConstants;
// import frc.robot.subsystems.templates.PositionSubsystem;

// public class LauncherWrist extends PositionSubsystem {


//     private static LauncherWrist m_instance = null;

//     private CANcoder m_CANCoder;

//     public LauncherWrist(PositionSubsystemConstants constants) {
//         super(constants);

//         m_CANCoder = new CANcoder(LauncherConstants.kWristCANCoderId);

//         zero(m_CANCoder.getAbsolutePosition().getValue());
//     }

//     public static LauncherWrist getInstance() {
//         if (m_instance == null) {
//             m_instance = new LauncherWrist(LauncherConstants.kLauncherWristConstants);
//         }

//         return m_instance;
//     }

//     @Override
//     public void subsystemPeriodic() {
//         LauncherWristState.FIELD_BASED_PITCH.setPosition(calculatePitch());
//         SmartDashboard.putNumber("Calculated shooter Pitch", m_currentState.getPosition());
//     }

//     @Override
//     public void outputTelemetry() {}

//     public double calculatePitch() {
//         // double distance = SwerveDrive.getInstance().getPose().getTranslation().getDistance(DriveConstants.kBlueSpeakerPosition);

//         // if (distance > 0 && distance < LauncherConstants.kDistancePitchMap.lastKey()) {
//         //     double lowerPitch = LauncherConstants.kDistancePitchMap.get(LauncherConstants.kDistancePitchMap.floorKey(distance));
//         //     double upperPitch = LauncherConstants.kDistancePitchMap.get(LauncherConstants.kDistancePitchMap.ceilingKey(distance));
//         //     return lowerPitch + (distance - Math.floor(distance)) * (upperPitch - lowerPitch);
//         // } else {
//         //     return 0;
//         // }

//         return 0;
//     }

//     public enum LauncherWristState implements PositionSubsystemState {
//         DOWN(0, 0, "Down"),
//         UP(45, 0, "Up"),
//         FIELD_BASED_PITCH(0, 0, "Field Based Pitch"),
//         TRANSITION(0, 0, "Transition"),
//         MANUAL(0, 0, "Manual");
    
//         private double position;
//         private double velocity;
//         private String name;
    
//         private LauncherWristState(double position, double velocity, String name) {
//           this.position = position;
//           this.velocity = velocity;
//           this.name = name;
//         }

//         @Override
//         public String getName() {
//             return name;
//         }

//         @Override
//         public double getVelocity() {
//             return velocity;
//         }

//         @Override
//         public void setVelocity(double velocity) {
//             this.velocity = velocity;
//         }

//         @Override
//         public double getPosition() {
//             return position;
//         }

//         @Override
//         public void setPosition(double position) {
//             this.position = position;
//         }
//     }
    
// }
