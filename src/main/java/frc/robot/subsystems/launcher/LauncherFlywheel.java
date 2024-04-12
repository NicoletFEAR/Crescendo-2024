package frc.robot.subsystems.launcher;


import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.templates.VelocitySubsystem;
import frc.lib.templates.SubsystemConstants.RevMotorType;
import frc.lib.templates.SubsystemConstants.SparkConstants;
import frc.lib.templates.SubsystemConstants.VelocitySubsystemConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.launcher.LauncherSuperstructure.LauncherConstants;

public class LauncherFlywheel extends VelocitySubsystem {

    private static LauncherFlywheel m_instance = null;

    public LauncherFlywheel(VelocitySubsystemConstants constants) {
        super(constants);
    }

    public static LauncherFlywheel getInstance() {
        if (m_instance == null) {
            m_instance = new LauncherFlywheel(LauncherFlywheelConstants.kLauncherFlywheelConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {
        // LauncherFlywheelState.PASS.setVelocity(new double[] {calculateAmpRPM(), calculateAmpRPM()});
        
        if (m_currentState == LauncherFlywheelState.FIELD_BASED_VELOCITY) {
            for (int i = 0; i < m_pidControllers.length; i++) {
                m_pidControllers[i].setReference(m_desiredState.getVelocity()[i], ControlType.kVelocity, m_constants.kDefaultSlot, m_arbFeedforward[i], ArbFFUnits.kVoltage);
            }
        }
    }

    @Override
    public void outputTelemetry() {
        
    }

    public double calculateAmpRPM() {
        double distance;

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            distance = RobotContainer.m_drivebase.getPose().getTranslation().getDistance(DriveConstants.kRedAmpPassPosition);
        } else {
            distance = RobotContainer.m_drivebase.getPose().getTranslation().getDistance(DriveConstants.kBlueAmpPassPosition);
        }
        
        if (distance > 0 && distance < LauncherConstants.kAmpDistanceRPMMap.lastKey()) {
            double lowerRPM = LauncherConstants.kAmpDistanceRPMMap.get(LauncherConstants.kAmpDistanceRPMMap.floorKey(distance));
            double upperRPM = LauncherConstants.kAmpDistanceRPMMap.get(LauncherConstants.kAmpDistanceRPMMap.ceilingKey(distance));
            return lowerRPM + (distance - Math.floor(distance)) * (upperRPM - lowerRPM);
        } else {
            return 0;
        }
    }

    public enum LauncherFlywheelState implements VelocitySubsystemState {
        OFF(new double[] {0, 0}),
        IDLE(new double[] {2000, 2000}),
        TESTING(new double[] {4000, 4000}),
        WING_NOTE_1(new double[] {4000, 4000}),
        WING_NOTE_2(new double[] {4000, 4000}),
        WING_NOTE_3(new double[] {4000, 4000}),
        LAUNCH_POS_1(new double[] {5000, 5000}),
        LAUNCH_POS_2(new double[] {5000, 5000}),
        LAUNCH_POS_3(new double[] {5000, 5000}),
        POOP_POS_1(new double[] {4000, 4000}),
        POOP_POS_2(new double[] {4000, 4000}),
        POOP_POS_3(new double[] {4000, 4000}),
        RUNNING(new double[] {4000, 4000}), // arbitrary testing value
        SUBWOOFER(new double[] {6000, 6000}), // used for when against the base of the speaker left used to be 4000
        PASS(new double[] {6300, 6300}), 
        PODIUM(new double[] {6200, 6200}), // used for when against the base of the PODIUM
        TRANSITION(new double[] {0, 0}),
        FIELD_BASED_VELOCITY(new double[] {5800, 5800}),
        INTAKING(new double[] {-500, -500}), // used when intaking through launch
        MANUAL(new double[] {0, 0});

    
        private double[] velocity;
    
        private LauncherFlywheelState(double[] velocity) {
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
        public void setVelocity(double[] velocity) {
            this.velocity = velocity;
        }
    }
    
    public class LauncherFlywheelConstants {
        public static final SparkConstants kTopLauncherFlywheelConstants = new SparkConstants();

        static {
            kTopLauncherFlywheelConstants.kID = MotorConstants.kLauncherTopFlywheelID;
            kTopLauncherFlywheelConstants.kRevMotorType = RevMotorType.CAN_SPARK_FLEX;
            kTopLauncherFlywheelConstants.kName = "Top Launcher Flywheel";
            kTopLauncherFlywheelConstants.kIdleMode = IdleMode.kCoast;
            kTopLauncherFlywheelConstants.kMotorType = MotorType.kBrushless;
            kTopLauncherFlywheelConstants.kCurrentLimit = 40; //80;
            kTopLauncherFlywheelConstants.kInverted = false;
            kTopLauncherFlywheelConstants.kKp = 0.0002; //0.00025;
            kTopLauncherFlywheelConstants.kKi = 0.0;
            kTopLauncherFlywheelConstants.kKd = 0.0002; //0.0;
            kTopLauncherFlywheelConstants.kKff = 0.000142; //0.00015;
        }

        public static final SparkConstants kBottomLauncherFlywheelConstants = new SparkConstants();

        static {
            kBottomLauncherFlywheelConstants.kID = MotorConstants.kLauncherBottomFlywheelID;
            kBottomLauncherFlywheelConstants.kRevMotorType = RevMotorType.CAN_SPARK_FLEX;
            kBottomLauncherFlywheelConstants.kName = "Bottom Launcher Flywheel";
            kBottomLauncherFlywheelConstants.kIdleMode = IdleMode.kCoast;
            kBottomLauncherFlywheelConstants.kMotorType = MotorType.kBrushless;
            kBottomLauncherFlywheelConstants.kCurrentLimit = 40;// 80;
            kBottomLauncherFlywheelConstants.kInverted = false;
            kBottomLauncherFlywheelConstants.kKp = 0.0002; //0.00025;
            kBottomLauncherFlywheelConstants.kKi = 0.0;
            kBottomLauncherFlywheelConstants.kKd = 0.0002; //0.0;
            kBottomLauncherFlywheelConstants.kKff = 0.000142; //0.00015;
        }

        public static final VelocitySubsystemConstants kLauncherFlywheelConstants =
            new VelocitySubsystemConstants();

        static {
            kLauncherFlywheelConstants.kSubsystemName = "Launcher Flywheel";
            kLauncherFlywheelConstants.kSuperstructureName = "Launcher";

            kLauncherFlywheelConstants.kSubsystemType = VelocitySubsystemType.LAUNCHER_FLYWHEEL;

            kLauncherFlywheelConstants.kMotorConstants = new SparkConstants[] {kTopLauncherFlywheelConstants, kBottomLauncherFlywheelConstants};

            kLauncherFlywheelConstants.kVelocityConversionFactor = 1.0; 

            kLauncherFlywheelConstants.kDefaultSlot = 0;

            kLauncherFlywheelConstants.kSetpointTolerance = 100;

            kLauncherFlywheelConstants.kInitialState = LauncherFlywheelState.OFF;
            kLauncherFlywheelConstants.kTransitionState = LauncherFlywheelState.TRANSITION;
            kLauncherFlywheelConstants.kManualState = LauncherFlywheelState.MANUAL;
        }
    }

}
