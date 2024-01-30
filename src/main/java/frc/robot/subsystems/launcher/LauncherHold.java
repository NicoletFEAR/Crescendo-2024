// package frc.robot.subsystems.launcher;

// import frc.robot.Constants.LauncherConstants;
// import frc.robot.subsystems.templates.SubsystemConstants.VoltageSubsystemConstants;
// import frc.robot.subsystems.templates.VoltageSubsystem;

// public class LauncherHold extends VoltageSubsystem {

//     private static LauncherHold m_instance = null;

//     protected LauncherHold(VoltageSubsystemConstants constants) {
//         super(constants);
//     }

//     public static LauncherHold getInstance() {
//         if (m_instance == null) {
//             m_instance = new LauncherHold(LauncherConstants.kLauncherHoldConstants);
//         }

//         return m_instance;
//     }


//     @Override
//     public void subsystemPeriodic() {}

//     @Override
//     public void outputTelemetry() {}

//     public enum LauncherHoldState implements VoltageSubsystemState {
//         OFF(0, "Off"),
//         FEEDING(3, "Up"),
//         OUTTAKING(-3, "Outtaking"),
//         TRANSITION(0, "Transition");
    
//         private double voltage;
//         private String name;
    
//         private LauncherHoldState(double voltage, String name) {
//           this.voltage = voltage;
//           this.name = name;
//         }

//         @Override
//         public String getName() {
//             return name;
//         }

//         @Override
//         public double getVoltage() {
//             return voltage;
//         }
//     }
    
// }
