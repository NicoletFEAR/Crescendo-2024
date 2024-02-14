package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.MotorConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeSuperstructure.IntakeConstants;
import frc.robot.subsystems.templates.MultiMotorPositionSubsystem.MultiMotorPositionSubsystemState;
import frc.robot.subsystems.templates.MultiMotorPositionSubsystem.MultiMotorPositionSubsystemType;
import frc.robot.subsystems.templates.MultiMotorPositionSubsystem;
import frc.robot.subsystems.templates.PositionSubsystem;
import frc.robot.subsystems.templates.SubsystemConstants.ManualControlMode;
import frc.robot.subsystems.templates.SubsystemConstants.MultiMotorPositionSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;

public class MultiElevatorLift extends MultiMotorPositionSubsystem {

    private static MultiElevatorLift m_instance = null;
    DigitalInput m_limitSwitch;

    public MultiElevatorLift(MultiMotorPositionSubsystemConstants constants) {
        super(constants);
        m_limitSwitch = new DigitalInput(IntakeConstants.elevatorLimitSwitchID);

        // if(m_limitSwitch.get()){
        //     zero(0);
        // }
    }

    public static MultiElevatorLift getInstance() {
        if (m_instance == null) {
            m_instance = new MultiElevatorLift(MultiElevatorLiftConstants.kMultiElevatorLiftConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {
    }

    @Override
    public void outputTelemetry() {
    }

    public enum MultiElevatorLiftState implements MultiMotorPositionSubsystemState {
        DOWN(new double[] {0, 0}, new double[] {0, 0}, "Down"),
        TRANSITION(new double[] {0, 0}, new double[] {0, 0}, "Transition"),
        AMP(new double[] {37.5, 37.5}, new double[] {0, 0}, "Amp"),
        MANUAL(new double[] {0, 0}, new double[] {0, 0}, "Manual");
    
        private double[] position;
        private double[] velocity;
        private String name;
    
        private MultiElevatorLiftState(double[] position, double[] velocity, String name) {
          this.position = position;
          this.velocity = velocity;
          this.name = name;
        }

        @Override
        public String getName() {
            return name;
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


public static class MultiElevatorLiftConstants {
    public static final SparkConstants[] kElevatorLiftMotorConstants = new SparkConstants[2];

    static {
    kElevatorLiftMotorConstants[0] = new SparkConstants();
    kElevatorLiftMotorConstants[0].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
    kElevatorLiftMotorConstants[0].kID = MotorConstants.kElevatorLiftRightID;
    kElevatorLiftMotorConstants[0].kName = "Elevator Lift Right";
    kElevatorLiftMotorConstants[0].kIdleMode = IdleMode.kBrake;
    kElevatorLiftMotorConstants[0].kMotorType = MotorType.kBrushless;
    kElevatorLiftMotorConstants[0].kCurrentLimit = 80;
    kElevatorLiftMotorConstants[0].kInverted = false;
    kElevatorLiftMotorConstants[0].kKp = 0.125;
    kElevatorLiftMotorConstants[0].kKi = 0.001;
    kElevatorLiftMotorConstants[0].kKd = 0.0;

    kElevatorLiftMotorConstants[1] = new SparkConstants();
    kElevatorLiftMotorConstants[1].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
    kElevatorLiftMotorConstants[1].kID = MotorConstants.kElevatorLiftLeftID;
    kElevatorLiftMotorConstants[1].kName = "Elevator Lift Left";
    kElevatorLiftMotorConstants[1].kIdleMode = IdleMode.kBrake;
    kElevatorLiftMotorConstants[1].kMotorType = MotorType.kBrushless;
    kElevatorLiftMotorConstants[1].kCurrentLimit = 80;
    kElevatorLiftMotorConstants[1].kInverted = true;
    kElevatorLiftMotorConstants[1].kKp = 0.125;
    kElevatorLiftMotorConstants[1].kKi = 0.001;
    kElevatorLiftMotorConstants[1].kKd = 0.0;
    }

    public static MultiMotorPositionSubsystemConstants kMultiElevatorLiftConstants = new MultiMotorPositionSubsystemConstants();

    static {
    // Name of the subsystem, for example "Launcher Flywheels"
    kMultiElevatorLiftConstants.kSubsystemName = "Elevator Lift"; 
    
    // Name of the subsystem, for example "Launcher"
    kMultiElevatorLiftConstants.kSuperstructureName = "Intake";

    // An enum which is in the template subsystem
    kMultiElevatorLiftConstants.kSubsystemType = MultiMotorPositionSubsystemType.ELEVATOR_LIFT;

    // An array of motors to be set to an array of positions
    // Instantiate these motor constants above this static block
    kMultiElevatorLiftConstants.kMotorConstants = kElevatorLiftMotorConstants;

    // Initial, Manual, and Transition state of the subsytem
    // This enum is in the Subsystem that extends the MultiMotorPositionSubsystem
    // You will have to create these states
    kMultiElevatorLiftConstants.kInitialState = MultiElevatorLiftState.DOWN;
    kMultiElevatorLiftConstants.kManualState = MultiElevatorLiftState.MANUAL;
    kMultiElevatorLiftConstants.kTransitionState = MultiElevatorLiftState.TRANSITION;

    // Home position of the motor
    kMultiElevatorLiftConstants.kHomePosition = 0.0;

    // Conversion factor for the motor output units
    // To find degrees: 360/gear ratio ex 360/100 for 100:1
    // For example for ratio 100:1 do 100
    kMultiElevatorLiftConstants.kPositionConversionFactor = 1.0; 

    // Tolerance for atSetpoint()
    kMultiElevatorLiftConstants.kSetpointTolerance = 1.5; 

    // PID Slot, make more if more than one set of pid constants are used
    kMultiElevatorLiftConstants.kDefaultSlot = 0; 

    // Max velocity and acceleration for trapezoidal motion profile
    kMultiElevatorLiftConstants.kMaxVelocity = 10.0; 
    kMultiElevatorLiftConstants.kMaxAcceleration = 5.0;

    // Max/Min positions the subsystem should be able to move
    kMultiElevatorLiftConstants.kMaxPosition = 38.;
    kMultiElevatorLiftConstants.kMinPosition = 0;

    // Enum which is found in SubsystemConstants
    kMultiElevatorLiftConstants.kManualControlMode = ManualControlMode.TRIGGERS;

    // Multiplied by controller inputs
    kMultiElevatorLiftConstants.kManualMultiplier = .05;

    // Deadband for controller
    kMultiElevatorLiftConstants.kManualDeadBand = .1;
    }
    
}


@Override
public void manualControl() {
    double m_throttle = 0;

    switch (m_constants.kManualControlMode) {
      case BUMPERS:
         m_throttle =
          RobotContainer.m_operatorController.getHID().getLeftBumper()
              ? -1
              : (RobotContainer.m_operatorController.getHID().getRightBumper() ? 1 : 0);
        break;
      case LEFT_X:
        m_throttle = RobotContainer.m_operatorController.getLeftX();
        break;
      case LEFT_Y:
        m_throttle = -RobotContainer.m_operatorController.getLeftY();
        break;
      case RIGHT_X:
        m_throttle = RobotContainer.m_operatorController.getRightX();
        break;
      case RIGHT_Y:
        m_throttle = -RobotContainer.m_operatorController.getRightY();
        break;
      case TRIGGERS:
        m_throttle = 
          RobotContainer.m_operatorController.getRightTriggerAxis()
              - RobotContainer.m_operatorController.getLeftTriggerAxis();
        break;
    }

    m_throttle = MathUtil.applyDeadband(m_throttle, m_constants.kManualDeadBand);

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
}
