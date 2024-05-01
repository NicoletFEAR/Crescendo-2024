package frc.robot.subsystems.intake;

import frc.lib.templates.VoltageSubsystem;
import frc.lib.templates.SubsystemConstants.RevMotorType;
import frc.lib.templates.SubsystemConstants.SparkConstants;
import frc.lib.templates.SubsystemConstants.VoltageSubsystemConstants;
import frc.robot.Constants.MotorConstants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeFlywheel extends VoltageSubsystem {

    private static IntakeFlywheel m_instance = null;

    
  

    protected IntakeFlywheel(VoltageSubsystemConstants constants) {
        super(constants);
    }

    public static IntakeFlywheel getInstance() {
        if (m_instance == null) {
            m_instance = new IntakeFlywheel(IntakeFlywheelConstants.kIntakeFlywheelConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {}

    @Override
    public void outputTelemetry() {}

    public enum IntakeFlywheelState implements VoltageSubsystemState {
        OFF(0),
        INTAKING(-12),
        SLOW_INTAKING(-8),
        INTAKE_TO_LAUNCH(-12),
        EJECTING(6),
        LAUNCH_TO_INTAKE(1.5),
        AMP(12),
        TRANSITION(0),
        MANUAL(0);
        
        private double voltage;
    
        private IntakeFlywheelState(double voltage) {
          this.voltage = voltage;
        }

        @Override
        public String getName() {
            return name();
        }

        @Override
        public double getVoltage() {
            return voltage;
        }

        // @Override
        // public void setVoltage(double voltage) {
        //     this.voltage = voltage;
        // }
    }

    

    public class IntakeFlywheelConstants {
        public static final SparkConstants kIntakeFlywheelLeaderConstants = new SparkConstants();

        static {
            kIntakeFlywheelLeaderConstants.kID = MotorConstants.kIntakeFlywheelID;
            kIntakeFlywheelLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kIntakeFlywheelLeaderConstants.kName = "Intake Flywheel";
            kIntakeFlywheelLeaderConstants.kIdleMode = IdleMode.kBrake;
            kIntakeFlywheelLeaderConstants.kMotorType = MotorType.kBrushless;
            kIntakeFlywheelLeaderConstants.kCurrentLimit = 80; // 60;
            kIntakeFlywheelLeaderConstants.kInverted = false;
            kIntakeFlywheelLeaderConstants.kKp = 0.01; // 0.00001
            kIntakeFlywheelLeaderConstants.kKi = 0.0;
            kIntakeFlywheelLeaderConstants.kKd = 0.0;
            kIntakeFlywheelLeaderConstants.kKff = 0.0001675;
        }

        public static final VoltageSubsystemConstants kIntakeFlywheelConstants =
            new VoltageSubsystemConstants();

        static {
            kIntakeFlywheelConstants.kSubsystemName = "Intake Flywheel";
            kIntakeFlywheelConstants.kSuperstructureName = "Intake";

            kIntakeFlywheelConstants.kSubsystemType = VoltageSubsystemType.INTAKE_FLYWHEELS;

            kIntakeFlywheelConstants.kLeaderConstants = kIntakeFlywheelLeaderConstants;
           
            kIntakeFlywheelConstants.kInitialState = IntakeFlywheelState.OFF;
        }
    }

    
}
