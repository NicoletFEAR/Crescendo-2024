// package frc.robot.subsystems;

// import frc.robot.Constants.FalconTestingConstants;
// import frc.robot.subsystems.templates.PositionSubsystem.PositionSubsystemState;
// import frc.robot.subsystems.templates.SubsystemConstants.TalonFXPositionSubsystemConstants;
// import frc.robot.subsystems.templates.TalonFXPositionSubsystem;

// public class FalconTestingStateMachine extends TalonFXPositionSubsystem{

//     private static FalconTestingStateMachine m_instance = null;

//     protected FalconTestingStateMachine(TalonFXPositionSubsystemConstants constants) {
//         super(constants);
//     }

//     public static FalconTestingStateMachine getInstance() {
//         if (m_instance == null) {
//             m_instance = new FalconTestingStateMachine(FalconTestingConstants.kFalconTestingConstants);
//         }

//         return m_instance;
//     }

//     @Override
//     public void subsystemPeriodic() {}

//     @Override
//     public void outputTelemetry() {}
    

//     public enum FalconTestingState implements PositionSubsystemState {
//         DOWN(0, 0, "Down"),
//         UP(50, 0, "Up"),
//         REALLY_UP(150, 0, "Really Up"),
//         REALLY_REALLY_UP(300, 0, "Really Really Up"),
//         TRANSITION(0, 0, "Transition"),
//         MANUAL(0, 0, "Manual");
    
//         private double position;
//         private double velocity;
//         private String name;
    
//         private FalconTestingState(double position, double velocity, String name) {
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
