// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Graber extends SubsystemBase {
  private TalonFX m_Graber;
  private TalonFXConfiguration configs;
  private MotionMagicVelocityVoltage m_request;


  private int graberID = 0;
  private double graberPGains = 0;
  private double graberIGains = 0;
  private double graberDGains = 0;
  private double currentLimit = 0;
  private double gearRatio = 0;

  private double corralIntakeSpeed = 0;
  private double corralOuttakeSpeed = 0;
  private double algaeIntakeSpeed = 0;
  private double algaeOuttakeSpeed = 0;
  /** Creates a new Graber. */
  public Graber() {
    m_Graber = new TalonFX(graberID);
    m_request = new MotionMagicVelocityVoltage(0).withSlot(0);

    configs = new TalonFXConfiguration();
    configs.Slot0.kP = graberPGains;
    configs.Slot0.kI = graberIGains;
    configs.Slot0.kD = graberDGains;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    configs.Feedback.SensorToMechanismRatio = gearRatio;



  }

  private void intakeCorral(double speed) {
    m_Graber.setControl(m_request.withVelocity(speed));
  }

  private void intakeAlgae(double speed) {
    m_Graber.setControl(m_request.withVelocity(speed));
  } 

  private double getCurrent() {
    return m_Graber.getSupplyCurrent().getValueAsDouble();
  }
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Graber Current", getCurrent());
  }
}
