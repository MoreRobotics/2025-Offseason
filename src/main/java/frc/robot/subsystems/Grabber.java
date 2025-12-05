// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
  private TalonFX m_Grabber;
  private TalonFXConfiguration configs;
  private VelocityVoltage m_request;


  private int graberID = 13;
  private double graberPGains = 0.4;
  private double graberIGains = 0;
  private double graberDGains = 0;
  private double currentLimit = 80;
  private double gearRatio = 0;

  public double corralIntakeSpeed = 20;
  private double corralOuttakeSpeed = 0;
  private double algaeIntakeSpeed = 0;
  private double algaeOuttakeSpeed = 0;
  /** Creates a new Graber. */
  public Grabber() {
    m_Grabber = new TalonFX(graberID);
    m_request = new VelocityVoltage(0).withSlot(0);

    configs = new TalonFXConfiguration();
    configs.Slot0.kP = graberPGains;
    configs.Slot0.kI = graberIGains;
    configs.Slot0.kD = graberDGains;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    configs.Feedback.SensorToMechanismRatio = gearRatio;
    m_Grabber.getConfigurator().apply(configs);



  }

  public void intakeCorral(double speed) {
    m_Grabber.setControl(m_request.withVelocity(speed));
  }

  private void intakeAlgae(double speed) {
    m_Grabber.setControl(m_request.withVelocity(speed));
  } 

  private double getCurrent() {
    return m_Grabber.getSupplyCurrent().getValueAsDouble();
  }
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Graber Current", getCurrent());
  }
}
