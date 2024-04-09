package frc.robot.subsystems.intake;

import frc.lib.templates.VoltageSubsystem;
import frc.lib.templates.SubsystemConstants.RevMotorType;
import frc.lib.templates.SubsystemConstants.SparkConstants;
import frc.lib.templates.SubsystemConstants.VoltageSubsystemConstants;
import frc.robot.Constants.MotorConstants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeHold extends VoltageSubsystem {

    private static IntakeHold m_instance = null;

    protected IntakeHold(VoltageSubsystemConstants constants) {
        super(constants);
    }

    public static IntakeHold getInstance() {
        if (m_instance == null) {
            m_instance = new IntakeHold(IntakeHoldConstants.kIntakeHoldConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {}

    @Override
    public void outputTelemetry() {}

    public enum IntakeHoldState implements VoltageSubsystemState {
        OFF(0),
        INTAKING(-9),
        LAUNCH_IN(4),
        INTAKE_TO_LAUNCH(-2),
        AMP(6),
        EJECTING(6);
        
        private double voltage;
    
        private IntakeHoldState(double voltage) {
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

    public class IntakeHoldConstants {
        public static final SparkConstants kIntakeHoldLeaderConstants = new SparkConstants();

        static {
            kIntakeHoldLeaderConstants.kID = MotorConstants.kIntakeHoldID;
            kIntakeHoldLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kIntakeHoldLeaderConstants.kName = "Intake Hold";
            kIntakeHoldLeaderConstants.kIdleMode = IdleMode.kBrake;
            kIntakeHoldLeaderConstants.kMotorType = MotorType.kBrushless;
            kIntakeHoldLeaderConstants.kCurrentLimit = 10; //80;
            kIntakeHoldLeaderConstants.kInverted = false;
            kIntakeHoldLeaderConstants.kKp = 0.00001;
            kIntakeHoldLeaderConstants.kKi = 0.0;
            kIntakeHoldLeaderConstants.kKd = 0.0;
            kIntakeHoldLeaderConstants.kKff = 0.0001675;
        }

        public static final VoltageSubsystemConstants kIntakeHoldConstants =
            new VoltageSubsystemConstants();

        static {
            kIntakeHoldConstants.kSubsystemName = "Intake Hold";
            kIntakeHoldConstants.kSuperstructureName = "Intake";

            kIntakeHoldConstants.kSubsystemType = VoltageSubsystemType.INTAKE_HOLD;

            kIntakeHoldConstants.kLeaderConstants = kIntakeHoldLeaderConstants;
           
            kIntakeHoldConstants.kInitialState = IntakeHoldState.OFF;
        }
    }

    
}
