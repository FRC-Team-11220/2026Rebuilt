// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  // private final SparkMax feederRoller;
  // private final SparkMax intakeLauncherRoller;
  private final TalonFX feederRoller;
  private final TalonFX intakeLauncherRoller;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    // intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
    // feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed);
    intakeLauncherRoller = new TalonFX(INTAKE_LAUNCHER_MOTOR_ID, new CANBus("rio"));
    feederRoller = new TalonFX(FEEDER_MOTOR_ID, new CANBus("rio"));
    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    // SparkMaxConfig feederConfig = new SparkMaxConfig();
    // feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    // feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    CurrentLimitsConfigs feederConfig = new CurrentLimitsConfigs();
    feederConfig.withSupplyCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.getConfigurator().apply(feederConfig);
    

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    CurrentLimitsConfigs launcherConfig = new CurrentLimitsConfigs();
    // launcherConfig.inverted(true);
    launcherConfig.withSupplyCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.getConfigurator().apply(launcherConfig);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double voltage) {
    intakeLauncherRoller.setVoltage(voltage);
  }

  // A method to set the voltage of the Feeder roller
  public void setFeederRoller(double voltage) {
    feederRoller.setVoltage(-1*voltage);
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
