// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private TalonFX m1_Elevator;
  private TalonFX m2_Elevator;
  private MotionMagicVoltage m_request;
  private Slot0Configs slotConfigs;

  int m1_ElevatorID = 0;
  int m2_ElevatorID = 0;
  double elevatorPGains = 0;
  double elevatorIGains = 0;
  double elevatorDGains = 0;
  double elevatorGGains = 0;
  

  /** Creates a new Elevator. */
  public Elevator() {
    m1_Elevator = new TalonFX(m1_ElevatorID);
    m2_Elevator = new TalonFX(m2_ElevatorID);
    m_request = new MotionMagicVoltage(0).withSlot(0);

    slotConfigs = new Slot0Configs().withGravityType(GravityTypeValue.Elevator_Static);
    slotConfigs.kP = elevatorPGains;
    slotConfigs.kI = elevatorIGains;
    slotConfigs.kD = elevatorDGains;
    slotConfigs.kG = elevatorGGains;



  }

public void setPosition() {
  m1_Elevator.setControl(m_request);
}
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
