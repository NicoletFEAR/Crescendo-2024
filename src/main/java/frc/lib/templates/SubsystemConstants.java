package frc.lib.templates;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.templates.MultiMotorPositionSubsystem.MultiMotorPositionSubsystemState;
import frc.lib.templates.MultiMotorPositionSubsystem.MultiMotorPositionSubsystemType;
import frc.lib.templates.PositionSubsystem.PositionSubsystemState;
import frc.lib.templates.PositionSubsystem.PositionSubsystemType;
import frc.lib.templates.VelocitySubsystem.VelocitySubsystemState;
import frc.lib.templates.VelocitySubsystem.VelocitySubsystemType;
import frc.lib.templates.VoltageSubsystem.VoltageSubsystemState;
import frc.lib.templates.VoltageSubsystem.VoltageSubsystemType;

public class SubsystemConstants {

  public static class SparkConstants {
    public String kName = "ERROR_ASSIGN_NAME";
    public RevMotorType kRevMotorType = null;
    public int kID = 0;
    public IdleMode kIdleMode = IdleMode.kBrake;
    public MotorType kMotorType = MotorType.kBrushless;
    public int kCurrentLimit = 0;
    public boolean kInverted = false;
    public double kKp = 0.0;
    public double kKi = 0.0;
    public double kKd = 0.0;

    public double kKff = 0.0; // Really only use this for velocity control

    // If you want to use custom feedforward
    public double kKs = 0.0;
    public double kKg = 0.0;
    public double kKv = 0.0;
    public double kKa = 0.0;
  }

  public static class PositionSubsystemConstants {

    // Subsystem Constants \\
    public String kSubsystemName = "ERROR_ASSIGN_A_NAME";
    public String kSuperstructureName = "ERROR_ASSIGN_A_NAME";

    public PositionSubsystemType kSubsystemType = null;

    public SparkConstants kLeaderConstants = new SparkConstants();
    public SparkConstants[] kFollowerConstants = new SparkConstants[0];

    public PositionSubsystemState kInitialState = null;
    public PositionSubsystemState kManualState = null;
    public PositionSubsystemState kTransitionState = null;

    // Servo Motor Subsystem Constants \\
    public double kHomePosition = 0.0;
    public double kPositionConversionFactor =
        1.0; // To find degrees: 360/gear ration ex 360/100 for 100:1

    public double kSetpointTolerance = 0.0; // Tolerance for atSetpoint()

    public int kDefaultSlot =
        0; // PID Slot, make more if more than one set of pid constants are used

    public double kMaxVelocity = 0.0; // Max velocity for motion profile
    public double kMaxAcceleration = 0.0; // Max acceleration for motion profile

    // Max/Min positions the subsystem should be able to move
    public double kMaxPosition = Double.POSITIVE_INFINITY;
    public double kMinPosition = Double.NEGATIVE_INFINITY;

    // Manual constants
    public ManualControlMode kManualControlMode = null;
    public double kManualMultiplier = 0;
    public double kManualDeadBand = 0;
  }

  public static class MultiMotorPositionSubsystemConstants {

    // Subsystem Constants \\
    public String kSubsystemName = "ERROR_ASSIGN_A_NAME";
    public String kSuperstructureName = "ERROR_ASSIGN_A_NAME";

    public MultiMotorPositionSubsystemType kSubsystemType = null;

    public SparkConstants[] kMotorConstants = new SparkConstants[0];

    public MultiMotorPositionSubsystemState kInitialState = null;
    public MultiMotorPositionSubsystemState kManualState = null;
    public MultiMotorPositionSubsystemState kTransitionState = null;

    // Servo Motor Subsystem Constants \\
    public double kHomePosition = 0.0;
    public double kPositionConversionFactor =
        1.0; // To find degrees: 360/gear ration ex 360/100 for 100:1

    public double kSetpointTolerance = 0.0; // Tolerance for atSetpoint()

    public int kDefaultSlot =
        0; // PID Slot, make more if more than one set of pid constants are used

    public double kMaxVelocity = 0.0; // Max velocity for motion profile
    public double kMaxAcceleration = 0.0; // Max acceleration for motion profile

    // Max/Min positions the subsystem should be able to move
    public double kMaxPosition = Double.POSITIVE_INFINITY;
    public double kMinPosition = Double.NEGATIVE_INFINITY;

    // Manual constants
    public ManualControlMode kManualControlMode = null;
    public double kManualMultiplier = 0;
    public double kManualDeadBand = 0;
  }

  public static class VoltageSubsystemConstants {
    public String kSubsystemName = "ERROR_ASSIGN_A_NAME";
    public String kSuperstructureName = "ERROR_ASSIGN_A_NAME";

    public VoltageSubsystemType kSubsystemType = null;

    public SparkConstants kLeaderConstants = new SparkConstants();
    public SparkConstants[] kFollowerConstants = new SparkConstants[0];

    public VoltageSubsystemState kInitialState = null;
  }

  public static class VelocitySubsystemConstants {
    public String kSubsystemName = "ERROR_ASSIGN_A_NAME";
    public String kSuperstructureName = "ERROR_ASSIGN_A_NAME";

    public VelocitySubsystemType kSubsystemType = null;

    public SparkConstants[] kMotorConstants = new SparkConstants[0];

    public VelocitySubsystemState kInitialState = null;
    public VelocitySubsystemState kTransitionState = null;
    public VelocitySubsystemState kManualState = null;

    public double kVelocityConversionFactor =
        1.0; // To find degrees: 360/gear ratio ex 360/100 for 100:1, just put 100 for 100:1

    public double kSetpointTolerance = 0.0; // Tolerance for atSetpoint()

    public int kDefaultSlot =
        0; // PID Slot, make more if more than one set of pid constants are used
  }

  public enum RevMotorType {
    CAN_SPARK_MAX,
    CAN_SPARK_FLEX
  }

  public static enum ManualControlMode {
    TRIGGERS,
    BUMPERS,
    LEFT_X,
    LEFT_Y,
    RIGHT_X,
    RIGHT_Y
  }
}

// VOLTAGE SUBSYSTEM CONSTANTS TEMPLATE

// public static final SparkConstants kExampleLeaderConstants = new SparkConstants();

// static {
//   kExampleLeaderConstants.kID = 7;
//   kExampleLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
//   kExampleLeaderConstants.kName = "Example Leader Flywheel";
//   kExampleLeaderConstants.kIdleMode = IdleMode.kBrake;
//   kExampleLeaderConstants.kMotorType = MotorType.kBrushless;
//   kExampleLeaderConstants.kCurrentLimit = 80;
//   kExampleLeaderConstants.kInverted = false;
// }

// public static final SparkConstants[] kExampleFollowerConstants = new SparkConstants[2];

// static {
//   kExampleFollowerConstants[0] = new SparkConstants();
//   kExampleFollowerConstants[0].kID = 7;
//   kExampleFollowerConstants[0].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
//   kExampleFollowerConstants[0].kName = "Example Follower Flywheel 1";
//   kExampleFollowerConstants[0].kIdleMode = IdleMode.kBrake;
//   kExampleFollowerConstants[0].kMotorType = MotorType.kBrushless;
//   kExampleFollowerConstants[0].kCurrentLimit = 80;
//   kExampleFollowerConstants[0].kInverted = false;

//   kExampleFollowerConstants[1] = new SparkConstants();
//   kExampleFollowerConstants[1].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
//   kExampleFollowerConstants[1].kID = 6;
//   kExampleFollowerConstants[1].kName = "Example Follower Flywheel 2";
//   kExampleFollowerConstants[1].kIdleMode = IdleMode.kBrake;
//   kExampleFollowerConstants[1].kMotorType = MotorType.kBrushless;
//   kExampleFollowerConstants[1].kCurrentLimit = 80;
//   kExampleFollowerConstants[1].kInverted = false;
// }

// public static VoltageSubsystemConstants kExampleVoltageSubsystemConstants = new VoltageSubsystemConstants();

// static {
//   // Name of the subsystem
//   kExampleVoltageSubsystemConstants.kSubsystemName = "Intake Wheels";

//   // Name of the superstructure encasing the subsystem
//   kExampleVoltageSubsystemConstants.kSuperstructureName = "Intake";

//   // Type of voltage subsystem, add this, for example INTAKE_WHEELS, to the enum in VoltageSubsystem
//   kExampleVoltageSubsystemConstants.kSubsystemType = null;

//   // Leader spark constants
//   kExampleVoltageSubsystemConstants.kLeaderConstants = kExampleLeaderConstants;

//   // Array of Spark constants to declare motors that follow the leader
//   kExampleVoltageSubsystemConstants.kFollowerConstants = kExampleFollowerConstants;

//   // Initial state of the subsystem, pulled from an enum that has to be created in the subclass of VoltageSubsystem
//   kExampleVoltageSubsystemConstants.kInitialState = null;
// }

// VELOCITY SUBSYSTEM CONSTANTS TEMPLATE

// public static final SparkConstants[] kExampleMotorConstants = new SparkConstants[2];

// static {
//   kExampleMotorConstants[0] = new SparkConstants();
//   kExampleMotorConstants[0].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
//   kExampleMotorConstants[0].kID = 7;
//   kExampleMotorConstants[0].kName = "Example Flywheel 1";
//   kExampleMotorConstants[0].kIdleMode = IdleMode.kBrake;
//   kExampleMotorConstants[0].kMotorType = MotorType.kBrushless;
//   kExampleMotorConstants[0].kCurrentLimit = 80;
//   kExampleMotorConstants[0].kInverted = false;
//   kExampleMotorConstants[0].kKp = 0.00001;
//   kExampleMotorConstants[0].kKi = 0.0;
//   kExampleMotorConstants[0].kKd = 0.0;
//   kExampleMotorConstants[0].kKff = 0.0001675;

//   kExampleMotorConstants[1] = new SparkConstants();
//   kExampleMotorConstants[1].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
//   kExampleMotorConstants[1].kID = 6;
//   kExampleMotorConstants[1].kName = "Example Flywheel 2";
//   kExampleMotorConstants[1].kIdleMode = IdleMode.kBrake;
//   kExampleMotorConstants[1].kMotorType = MotorType.kBrushless;
//   kExampleMotorConstants[1].kCurrentLimit = 80;
//   kExampleMotorConstants[1].kInverted = false;
//   kExampleMotorConstants[1].kKp = 0.00001;
//   kExampleMotorConstants[1].kKi = 0.0;
//   kExampleMotorConstants[1].kKd = 0.0;
//   kExampleMotorConstants[1].kKff = 0.0001675;
// }

// public static VelocitySubsystemConstants kExamplePositionSubsystemConstants = new VelocitySubsystemConstants();

// static {

// // Name of the subsystem
// kExamplePositionSubsystemConstants.kSubsystemName = "Launcher Flywheels";

// // Name of the superstructure containing this subsystem
// kExamplePositionSubsystemConstants.kSuperstructureName = "Launcher";

// // Enum containing the types are in the velocity subsystem template, you need to add this subsystem to the list
// kExamplePositionSubsystemConstants.kSubsystemType = null;

// // Array of motor constants
// // Make as long as you have motors
// kExamplePositionSubsystemConstants.kMotorConstants = kExampleMotorConstants;

// // Select states for use in the velocity subsystem
// // Enum containing these states need to be created in the subclass of the velocity subsystem
// kExamplePositionSubsystemConstants.kInitialState = null;
// kExamplePositionSubsystemConstants.kTransitionState = null;
// kExamplePositionSubsystemConstants.kManualState = null;

// // Conversion factor for the encoders
// // Multiplied by native units
// // Native units = rpm
// kExamplePositionSubsystemConstants.kVelocityConversionFactor = 1.0;

// // Tolerance for atSetpoint()
// kExamplePositionSubsystemConstants.kSetpointTolerance = 0.0; 

// // PID Slot, make more if more than one set of pid constants are used
// kExamplePositionSubsystemConstants.kDefaultSlot = 0; 
// }

// POSITION SUBSYSTEM CONSTANTS TEMPLATE

// public static final SparkConstants kExampleLeaderConstants = new SparkConstants();

// static {
//   kExampleLeaderConstants.kID = 8;
//   kExampleLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
//   kExampleLeaderConstants.kName = "Example Leader";
//   kExampleLeaderConstants.kIdleMode = IdleMode.kBrake;
//   kExampleLeaderConstants.kMotorType = MotorType.kBrushless;
//   kExampleLeaderConstants.kCurrentLimit = 80;
//   kExampleLeaderConstants.kInverted = false;
//   kExampleLeaderConstants.kKp = 0.1;
//   kExampleLeaderConstants.kKi = 0.0;
//   kExampleLeaderConstants.kKd = 0.0;
// }

// public static final SparkConstants[] kExampleFollowerConstants = new SparkConstants[2];

// static {
//   kExampleFollowerConstants[0] = new SparkConstants();
//   kExampleFollowerConstants[0].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
//   kExampleFollowerConstants[0].kID = 7;
//   kExampleFollowerConstants[0].kName = "Example Follower 1";
//   kExampleFollowerConstants[0].kIdleMode = IdleMode.kBrake;
//   kExampleFollowerConstants[0].kMotorType = MotorType.kBrushless;
//   kExampleFollowerConstants[0].kCurrentLimit = 80;
//   kExampleFollowerConstants[0].kInverted = false;
//   kExampleFollowerConstants[0].kKp = 0.1;
//   kExampleFollowerConstants[0].kKi = 0.0;
//   kExampleFollowerConstants[0].kKd = 0.0;

//   kExampleFollowerConstants[1] = new SparkConstants();
//   kExampleFollowerConstants[1].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
//   kExampleFollowerConstants[1].kID = 6;
//   kExampleFollowerConstants[1].kName = "Example Follower 2";
//   kExampleFollowerConstants[1].kIdleMode = IdleMode.kBrake;
//   kExampleFollowerConstants[1].kMotorType = MotorType.kBrushless;
//   kExampleFollowerConstants[1].kCurrentLimit = 80;
//   kExampleFollowerConstants[1].kInverted = false;
//   kExampleFollowerConstants[1].kKp = 0.1;
//   kExampleFollowerConstants[1].kKi = 0.0;
//   kExampleFollowerConstants[1].kKd = 0.0;
// }

// public static PositionSubsystemConstants kExamplePositionSubsystemConstants = new PositionSubsystemConstants();

// static {
//   // Name of the subsystem, for example "Launcher Flywheels"
//   kExamplePositionSubsystemConstants.kSubsystemName = "Example Multi Motor Position Subsystem"; 

//   // Name of the subsystem, for example "Launcher"
//   kExamplePositionSubsystemConstants.kSuperstructureName = "Example Multi Motor Position Superstructure";

//   // An enum which is in the template subsystem
//   kExamplePositionSubsystemConstants.kSubsystemType = null;

//   // The main motor constants
//   // Instantiate these motor constants above this static block
//   kExamplePositionSubsystemConstants.kLeaderConstants = kExampleLeaderConstants;

//   // An array of motor constants that follow the leader
//   // Instantiate these motor constants above this static block
//   kExamplePositionSubsystemConstants.kFollowerConstants = kExampleFollowerConstants;

//   // Initial, Manual, and Transition state of the subsytem
//   // This enum is in the Subsystem that extends the MultiMotorPositionSubsystem
//   // You will have to create these states
//   kExamplePositionSubsystemConstants.kInitialState = null;
//   kExamplePositionSubsystemConstants.kManualState = null;
//   kExamplePositionSubsystemConstants.kTransitionState = null;

//   // Home position of the motor
//   kExamplePositionSubsystemConstants.kHomePosition = 0.0;

//   // Conversion factor for the motor output units
//   // To find degrees: 360/gear ratio ex 360/100 for 100:1
//   // For example for ratio 100:1 do 100
//   kExamplePositionSubsystemConstants.kPositionConversionFactor = 1.0; 

//   // Tolerance for atSetpoint()
//   kExamplePositionSubsystemConstants.kSetpointTolerance = 0.0; 

//   // PID Slot, make more if more than one set of pid constants are used
//   kExamplePositionSubsystemConstants.kDefaultSlot = 0; 

//   // Max velocity and acceleration for trapezoidal motion profile
//   kExamplePositionSubsystemConstants.kMaxVelocity = 0.0; 
//   kExamplePositionSubsystemConstants.kMaxAcceleration = 0.0;

//   // Max/Min positions the subsystem should be able to move
//   kExamplePositionSubsystemConstants.kMaxPosition = Double.POSITIVE_INFINITY;
//   kExamplePositionSubsystemConstants.kMinPosition = Double.NEGATIVE_INFINITY;

//   // Enum which is found in SubsystemConstants
//   kExamplePositionSubsystemConstants.kManualControlMode = null;

//   // Multiplied by controller inputs
//   kExamplePositionSubsystemConstants.kManualMultiplier = 1;

//   // Deadband for controller
//   kExamplePositionSubsystemConstants.kManualDeadBand = .1;
// }

// MULTI MOTOR POSITION SUBSYSTEM CONSTANTS TEMPLATE

// public static final SparkConstants[] kExampleMotorConstants = new SparkConstants[2];

// static {
//   kExampleMotorConstants[0] = new SparkConstants();
//   kExampleMotorConstants[0].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
//   kExampleMotorConstants[0].kID = 7;
//   kExampleMotorConstants[0].kName = "Example Motor 1";
//   kExampleMotorConstants[0].kIdleMode = IdleMode.kBrake;
//   kExampleMotorConstants[0].kMotorType = MotorType.kBrushless;
//   kExampleMotorConstants[0].kCurrentLimit = 80;
//   kExampleMotorConstants[0].kInverted = false;
//   kExampleMotorConstants[0].kKp = 0.1;
//   kExampleMotorConstants[0].kKi = 0.0;
//   kExampleMotorConstants[0].kKd = 0.0;

//   kExampleMotorConstants[1] = new SparkConstants();
//   kExampleMotorConstants[1].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
//   kExampleMotorConstants[1].kID = 6;
//   kExampleMotorConstants[1].kName = "Example Motor 2";
//   kExampleMotorConstants[1].kIdleMode = IdleMode.kBrake;
//   kExampleMotorConstants[1].kMotorType = MotorType.kBrushless;
//   kExampleMotorConstants[1].kCurrentLimit = 80;
//   kExampleMotorConstants[1].kInverted = false;
//   kExampleMotorConstants[1].kKp = 0.1;
//   kExampleMotorConstants[1].kKi = 0.0;
//   kExampleMotorConstants[1].kKd = 0.0;
// }

// public static MultiMotorPositionSubsystemConstants kExampleMultiMotorPositionSubsystemConstants = new MultiMotorPositionSubsystemConstants();

// static {
//   // Name of the subsystem, for example "Launcher Flywheels"
//   kExampleMultiMotorPositionSubsystemConstants.kSubsystemName = "Example Multi Motor Position Subsystem"; 
  
//   // Name of the subsystem, for example "Launcher"
//   kExampleMultiMotorPositionSubsystemConstants.kSuperstructureName = "Example Multi Motor Position Superstructure";

//   // An enum which is in the template subsystem
//   kExampleMultiMotorPositionSubsystemConstants.kSubsystemType = null;

//   // An array of motors to be set to an array of positions
//   // Instantiate these motor constants above this static block
//   kExampleMultiMotorPositionSubsystemConstants.kMotorConstants = kExampleMotorConstants;

//   // Initial, Manual, and Transition state of the subsytem
//   // This enum is in the Subsystem that extends the MultiMotorPositionSubsystem
//   // You will have to create these states
//   kExampleMultiMotorPositionSubsystemConstants.kInitialState = null;
//   kExampleMultiMotorPositionSubsystemConstants.kManualState = null;
//   kExampleMultiMotorPositionSubsystemConstants.kTransitionState = null;

//   // Home position of the motor
//   kExampleMultiMotorPositionSubsystemConstants.kHomePosition = 0.0;

//   // Conversion factor for the motor output units
//   // To find degrees: 360/gear ratio ex 360/100 for 100:1
//   // For example for ratio 100:1 do 100
//   kExampleMultiMotorPositionSubsystemConstants.kPositionConversionFactor = 1.0; 

//   // Tolerance for atSetpoint()
//   kExampleMultiMotorPositionSubsystemConstants.kSetpointTolerance = 0.0; 

//   // PID Slot, make more if more than one set of pid constants are used
//   kExampleMultiMotorPositionSubsystemConstants.kDefaultSlot = 0; 

//   // Max velocity and acceleration for trapezoidal motion profile
//   kExampleMultiMotorPositionSubsystemConstants.kMaxVelocity = 0.0; 
//   kExampleMultiMotorPositionSubsystemConstants.kMaxAcceleration = 0.0;

//   // Max/Min positions the subsystem should be able to move
//   kExampleMultiMotorPositionSubsystemConstants.kMaxPosition = Double.POSITIVE_INFINITY;
//   kExampleMultiMotorPositionSubsystemConstants.kMinPosition = Double.NEGATIVE_INFINITY;

//   // Enum which is found in SubsystemConstants
//   kExampleMultiMotorPositionSubsystemConstants.kManualControlMode = null;

//   // Multiplied by controller inputs
//   kExampleMultiMotorPositionSubsystemConstants.kManualMultiplier = 1;

//   // Deadband for controller
//   kExampleMultiMotorPositionSubsystemConstants.kManualDeadBand = .1;
// }