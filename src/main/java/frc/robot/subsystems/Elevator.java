// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private TalonFX m1_Elevator;
  private TalonFX m2_Elevator;
  private MotionMagicVoltage m_request;
  private Follower m_Follower;
  private TalonFXConfiguration configs;

  private int m1_ElevatorID = 0;
  private int m2_ElevatorID = 12;
  private double elevatorPGains = 0;
  private double elevatorIGains = 0;
  private double elevatorDGains = 0;
  private double elevatorGGains = 0;
  private double elevatorHeight = 39;
  private double currentLimit = 0;
  private double target = 0;

  /** Creates a new Elevator. */
  public Elevator() {
    m1_Elevator = new TalonFX(m1_ElevatorID);
    m2_Elevator = new TalonFX(m2_ElevatorID);
    m_request = new MotionMagicVoltage(0).withSlot(0);
    m_Follower = new Follower(m1_ElevatorID, false);

    configs = new TalonFXConfiguration();
    configs.Slot0.kP = elevatorPGains;
    configs.Slot0.kI = elevatorIGains;
    configs.Slot0.kD = elevatorDGains;
    configs.Slot0.kG = elevatorGGains;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = elevatorHeight;
    configs.Feedback.SensorToMechanismRatio = Constants.ELEVATOR_ROTATIONS_TO_INCHES;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    

    m1_Elevator.getConfigurator().apply(configs);
    m2_Elevator.getConfigurator().apply(configs);
    m1_Elevator.setPosition(0);

  }

  public void moveElevator(double setpoint){
    target = setpoint;
    m1_Elevator.setControl(m_request.withPosition(setpoint));
    m2_Elevator.setControl(m_Follower.withMasterID(m1_ElevatorID));
  }



  private double getMotor1Position() {
    return m1_Elevator.getPosition().getValueAsDouble();
  }

  private double getMotor2Position() {
    return m2_Elevator.getPosition().getValueAsDouble();
  }

  private double getMotor1Current() {
    return m1_Elevator.getSupplyCurrent().getValueAsDouble();
  }

  private double getMotor2Current() {
    return m2_Elevator.getSupplyCurrent().getValueAsDouble();
  }


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Motor 1 Position", getMotor1Position());
    SmartDashboard.putNumber("Elevator Motor 2 Position", getMotor2Position());
    SmartDashboard.putNumber("Elevator Setpoint", target);
    SmartDashboard.putNumber("Elevator Motor 1 Current", getMotor1Current());
    SmartDashboard.putNumber("Elevator Motor 2 Current", getMotor2Current());

    
  }
}
