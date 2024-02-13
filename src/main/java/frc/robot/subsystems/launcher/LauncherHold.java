// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.templates.VoltageSubsystem;
import frc.robot.subsystems.templates.SubsystemConstants.VoltageSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;

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
        OFF(0, "Off"),
        LAUNCHING(12, "Launching"),
        INTAKING(-6, "Intaking");
        
        private double voltage;
        private String name;
    
        private LauncherHoldState(double voltage, String name) {
          this.voltage = voltage;
          this.name = name;
        }

        @Override
        public String getName() {
            return name;
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
            kLauncherHoldLeaderConstants.kCurrentLimit = 80;
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
