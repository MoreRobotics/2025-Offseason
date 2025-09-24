// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private TalonFX m_Wrist;
  private MotionMagicVoltage m_Request;
  private TalonFXConfiguration configs;

  private int m_WristID = 0;
  private double wristPGains = 0;
  private double wristIGains = 0;
  private double wristDGains = 0;
  private double forwardLimit = 0;
  private double reverseLimit = 0;
  private double gearRatio = 0;
  private double currentLimit = 0;
  private double target = 0;
  private double safePositionCurrentLimit = 0;



  /** Creates a new Wrist. */
  public Wrist() {
    m_Wrist = new TalonFX(m_WristID);

    m_Request = new MotionMagicVoltage(null).withSlot(0);

    configs = new TalonFXConfiguration();
    configs.Slot0.kP = wristPGains;
    configs.Slot0.kI = wristIGains;
    configs.Slot0.kD = wristDGains;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;
    configs.Feedback.SensorToMechanismRatio = gearRatio;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

  }

  private void setPosition() {
    m_Wrist.setControl(m_Request.withPosition(null));
  }

  private double getWristCurrent() {
    return m_Wrist.getSupplyCurrent().getValueAsDouble();
  }

  private void setSafePosition() {
    m_Wrist.setVoltage(3);
    if(getWristCurrent()>safePositionCurrentLimit) {
      m_Wrist.setVoltage(0);
    }
  }

  private double getWristPositionInDegrees() {
    return m_Wrist.getPosition().getValueAsDouble() * 360;
  }

  private void setWristPosition() {
    m_Wrist.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Current", getWristCurrent());
    SmartDashboard.putNumber("Wrist Position", getWristPositionInDegrees());

  }
}
