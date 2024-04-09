package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.MotorConstants;
import frc.lib.templates.MultiMotorPositionSubsystem;
import frc.lib.templates.SubsystemConstants.ManualControlMode;
import frc.lib.templates.SubsystemConstants.MultiMotorPositionSubsystemConstants;
import frc.lib.templates.SubsystemConstants.RevMotorType;
import frc.lib.templates.SubsystemConstants.SparkConstants;
import frc.robot.RobotContainer;

public class ElevatorLift extends MultiMotorPositionSubsystem {

    private static ElevatorLift m_instance = null;
    DigitalInput m_limitSwitch;

    public ElevatorLift(MultiMotorPositionSubsystemConstants constants) {
      super(constants);
    }

    public static ElevatorLift getInstance() {
        if (m_instance == null) {
            m_instance = new ElevatorLift(ElevatorLiftConstants.kElevatorLiftConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {}

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void manualControl() {
    double m_throttle = 
        MathUtil.applyDeadband(RobotContainer.m_operatorController.getRightTriggerAxis()
            - RobotContainer.m_operatorController.getLeftTriggerAxis(), m_constants.kManualDeadBand);

    if (m_currentState != m_constants.kManualState){
      m_constants.kManualState.setPosition(getPosition()[0], 0);
      m_constants.kManualState.setPosition(getPosition()[1], 1);
    }

    if (Math.abs(m_throttle) > 0 && m_profileStartTime == -1) {
      m_desiredState = m_constants.kManualState;
      m_currentState = m_constants.kManualState;

      m_throttle *= m_constants.kManualMultiplier;

      double intendedPosition = MathUtil.clamp(
              m_constants.kManualState.getPosition()[0] + m_throttle,
              m_constants.kMinPosition,
              m_constants.kMaxPosition);

      if (intendedPosition != m_constants.kManualState.getPosition()[0]) {
        m_constants.kManualState.setPosition(intendedPosition, 0);
        m_constants.kManualState.setPosition(intendedPosition, 1);
        if (m_profileStartTime == -1) {
          holdPosition();
        }
      }
    }
  }

    public enum ElevatorLiftState implements MultiMotorPositionSubsystemState {
      STOWED(new double[] {0, 0}, new double[] {0, 0}),
      TRANSITION(new double[] {0, 0}, new double[] {0, 0}),
      AMP(new double[] {41, 41}, new double[] {0, 0}),
      CLIMB(new double[] {80, 80}, new double[] {0, 0}),
      MANUAL(new double[] {0, 0}, new double[] {0, 0});
  
      private double[] position;
      private double[] velocity;
  
      private ElevatorLiftState(double[] position, double[] velocity) {
        this.position = position;
        this.velocity = velocity;
      }

      @Override
      public String getName() {
          return name();
      }

      @Override
      public double[] getVelocity() {
          return velocity;
      }

      @Override
      public void setVelocity(double velocity, int index) {
          this.velocity[index] = velocity;
      }

      @Override
      public double[] getPosition() {
          return position;
      }

      @Override
      public void setPosition(double position, int index) {
          this.position[index] = position;
      }
    }


public static class ElevatorLiftConstants {
    public static final SparkConstants[] kElevatorLiftMotorConstants = new SparkConstants[2];

    static {
    kElevatorLiftMotorConstants[0] = new SparkConstants();
    kElevatorLiftMotorConstants[0].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
    kElevatorLiftMotorConstants[0].kID = MotorConstants.kElevatorLiftRightID;
    kElevatorLiftMotorConstants[0].kName = "Elevator Lift Right";
    kElevatorLiftMotorConstants[0].kIdleMode = IdleMode.kBrake;
    kElevatorLiftMotorConstants[0].kMotorType = MotorType.kBrushless;
    kElevatorLiftMotorConstants[0].kCurrentLimit = 20; //80;
    kElevatorLiftMotorConstants[0].kInverted = false;
    kElevatorLiftMotorConstants[0].kKp = 0.14;
    kElevatorLiftMotorConstants[0].kKi = 0.0;
    kElevatorLiftMotorConstants[0].kKd = 0.0;

    kElevatorLiftMotorConstants[1] = new SparkConstants();
    kElevatorLiftMotorConstants[1].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
    kElevatorLiftMotorConstants[1].kID = MotorConstants.kElevatorLiftLeftID;
    kElevatorLiftMotorConstants[1].kName = "Elevator Lift Left";
    kElevatorLiftMotorConstants[1].kIdleMode = IdleMode.kBrake;
    kElevatorLiftMotorConstants[1].kMotorType = MotorType.kBrushless;
    kElevatorLiftMotorConstants[1].kCurrentLimit = 20; //80;
    kElevatorLiftMotorConstants[1].kInverted = true;
    kElevatorLiftMotorConstants[1].kKp = 0.14;
    kElevatorLiftMotorConstants[1].kKi = 0.0;
    kElevatorLiftMotorConstants[1].kKd = 0.0;
    }

    public static MultiMotorPositionSubsystemConstants kElevatorLiftConstants = new MultiMotorPositionSubsystemConstants();

    static {
    // Name of the subsystem, for example "Launcher Flywheels"
    kElevatorLiftConstants.kSubsystemName = "Elevator Lift"; 
    
    // Name of the subsystem, for example "Launcher"
    kElevatorLiftConstants.kSuperstructureName = "Intake";

    // An enum which is in the template subsystem
    kElevatorLiftConstants.kSubsystemType = MultiMotorPositionSubsystemType.ELEVATOR_LIFT;

    // An array of motors to be set to an array of positions
    // Instantiate these motor constants above this static block
    kElevatorLiftConstants.kMotorConstants = kElevatorLiftMotorConstants;

    // Initial, Manual, and Transition state of the subsytem
    // This enum is in the Subsystem that extends the MultiMotorPositionSubsystem
    // You will have to create these states
    kElevatorLiftConstants.kInitialState = ElevatorLiftState.STOWED;
    kElevatorLiftConstants.kManualState = ElevatorLiftState.MANUAL;
    kElevatorLiftConstants.kTransitionState = ElevatorLiftState.TRANSITION;

    // Home position of the motor
    kElevatorLiftConstants.kHomePosition = 0.0;

    // Conversion factor for the motor output units
    // To find degrees: 360/gear ratio ex 360/100 for 100:1
    // For example for ratio 100:1 do 100
    kElevatorLiftConstants.kPositionConversionFactor = 1.0; 

    // Tolerance for atSetpoint()
    kElevatorLiftConstants.kSetpointTolerance = 1.5; 

    // PID Slot, make more if more than one set of pid constants are used
    kElevatorLiftConstants.kDefaultSlot = 0; 

    // Max velocity and acceleration for trapezoidal motion profile
    kElevatorLiftConstants.kMaxVelocity = 500; 
    kElevatorLiftConstants.kMaxAcceleration = 600;

    // Max/Min positions the subsystem should be able to move
    kElevatorLiftConstants.kMaxPosition = 80;
    kElevatorLiftConstants.kMinPosition = 0;

    // Enum which is found in SubsystemConstants
    kElevatorLiftConstants.kManualControlMode = ManualControlMode.TRIGGERS;

    // Multiplied by controller inputs
    kElevatorLiftConstants.kManualMultiplier = 2; // 1.5

    // Deadband for controller
    kElevatorLiftConstants.kManualDeadBand = .1;
    }
    
}
}
