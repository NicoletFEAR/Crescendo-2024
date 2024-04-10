// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.templates.VoltageSubsystem;
import frc.lib.templates.SubsystemConstants.RevMotorType;
import frc.lib.templates.SubsystemConstants.SparkConstants;
import frc.lib.templates.SubsystemConstants.VoltageSubsystemConstants;
import frc.robot.Constants.MotorConstants;

public class LauncherHold extends VoltageSubsystem {

    private static LauncherHold m_instance = null;

    protected LauncherHold(VoltageSubsystemConstants constants) {
        super(constants);
    }

    public static LauncherHold getInstance() {
        if (m_instance == null) {
            m_instance = new LauncherHold(LauncherHoldConstants.kLauncherHoldConstants);
        }

        return m_instance;
    }


    @Override
    public void subsystemPeriodic() {}

    @Override
    public void outputTelemetry() {}

    public enum LauncherHoldState implements VoltageSubsystemState {
        OFF(0),
        LAUNCHING(-1.5),
        ADJUST_NOTE_IN(2.5),
        ADJUST_NOTE_OUT(-2.5),
        THRU_LAUNCHER_INTAKING(6),
        PASS(-12),
        THRU_INTAKE_INTAKING(-1.5),
        THRU_INTAKE_EJECTING(3);
        
        private double voltage;
    
        private LauncherHoldState(double voltage) {
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

    public class LauncherHoldConstants {
        public static final SparkConstants kLauncherHoldLeaderConstants = new SparkConstants();
        static {
            kLauncherHoldLeaderConstants.kID = MotorConstants.kLauncherHoldID;
            kLauncherHoldLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kLauncherHoldLeaderConstants.kName = "Launcher Hold";
            kLauncherHoldLeaderConstants.kIdleMode = IdleMode.kBrake;
            kLauncherHoldLeaderConstants.kMotorType = MotorType.kBrushless;
            kLauncherHoldLeaderConstants.kCurrentLimit = 20; //80;
            kLauncherHoldLeaderConstants.kInverted = false;
        }

        public static VoltageSubsystemConstants kLauncherHoldConstants = new VoltageSubsystemConstants();

        static {
        // Name of the subsystem
        kLauncherHoldConstants.kSubsystemName = "Launcher Hold";

        // Name of the superstructure encasing the subsystem
        kLauncherHoldConstants.kSuperstructureName = "Launcher";
        // Type of voltage subsystem, add this, for example INTAKE_WHEELS, to the enum in VoltageSubsystem
        kLauncherHoldConstants.kSubsystemType = VoltageSubsystemType.LAUNCHER_HOLD;
        // Leader spark constants
        kLauncherHoldConstants.kLeaderConstants = kLauncherHoldLeaderConstants;

        // Array of Spark constants to declare motors that follow the leader
        // kLauncherHoldConstants.kFollowerConstants = kExampleFollowerConstants; - No followers

        // Initial state of the subsystem, pulled from an enum that has to be created in the subclass of VoltageSubsystem
        kLauncherHoldConstants.kInitialState = LauncherHoldState.OFF;
        }
    }
            
}
