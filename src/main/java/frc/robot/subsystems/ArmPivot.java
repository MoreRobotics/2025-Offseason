// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.LimitSwitchConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivot extends SubsystemBase {
  private TalonFX m1_ArmPivot;
  private TalonFX m2_ArmPivot;
  private TalonFX m3_ArmPivot;
  private CANcoder e_armPivot;
  private MotionMagicVoltage m_Request;
  private CANcoderConfiguration e_Configs;
  private TalonFXConfiguration configs;
  private Follower m_Follow;


  private int m1_ArmPivotID = 0;
  private int m2_ArmPivotID = 0;
  private int m3_ArmPivotID = 0;
  private int e_armPivotID = 0;

  private double armPivotP = 0;
  private double armPivotI = 0;
  private double armPivotD = 0;
  private double forwardLimit = 0;
  private double reverseLimit = 0;
  private double gearRatio = 0;
  private double currentLimit = 0;
  private double magnetOffset = 0;
  private double target = 0;


  /** Creates a new ArmPivot. */
  public ArmPivot() {
    m1_ArmPivot = new TalonFX(m1_ArmPivotID);
    m2_ArmPivot = new TalonFX(m2_ArmPivotID);
    m3_ArmPivot = new TalonFX(m3_ArmPivotID);
    e_armPivot = new CANcoder(e_armPivotID);
    
    m_Request = new MotionMagicVoltage(null).withSlot(0);
    m_Follow = new Follower(m1_ArmPivotID, false);

    configs = new TalonFXConfiguration();
    configs.Slot0.kP = armPivotP;
    configs.Slot0.kI = armPivotI;
    configs.Slot0.kD = armPivotD;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;
    configs.Feedback.SensorToMechanismRatio = gearRatio;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    e_Configs = new CANcoderConfiguration();
    e_Configs.MagnetSensor.MagnetOffset = magnetOffset;
    e_Configs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    m1_ArmPivot.getConfigurator().apply(configs);
    m2_ArmPivot.getConfigurator().apply(configs);
    m3_ArmPivot.getConfigurator().apply(configs);


    e_armPivot.getConfigurator().apply(e_Configs);



    setInternalEncoder();


  }

  private void setInternalEncoder() {
    m1_ArmPivot.setPosition(getCANCoderInDegrees());
  }

  public void setArmPivotPosition(double setpoint) {
    target = setpoint;
    m1_ArmPivot.setControl(m_Request.withPosition(target));
    m2_ArmPivot.setControl(m_Follow.withMasterID(m1_ArmPivotID));
    m3_ArmPivot.setControl(m_Follow.withMasterID(m1_ArmPivotID));
  }

  private double getCANCoderInDegrees() {
    return e_armPivot.getPosition().getValueAsDouble() * 360;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmPivot CANCoder in degrees", getCANCoderInDegrees());
    
  }
}
