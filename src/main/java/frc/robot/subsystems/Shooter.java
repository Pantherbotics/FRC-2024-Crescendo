// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  TalonFX rightShooterWheel = new TalonFX(Constants.kRightShooterID);
  TalonFX leftShooterWheel = new TalonFX(Constants.kLeftShooterID);

  TalonFX leftWrist = new TalonFX(Constants.kLeftWristID);

  CANSparkMax rightShooterIntake = new CANSparkMax(Constants.kRightShooterIntakeID, MotorType.kBrushless);
  CANSparkMax leftShooterIntake = new CANSparkMax(Constants.kLeftShooterIntakeID, MotorType.kBrushless);
  
  AnalogInput distanceSensor = new AnalogInput(Constants.kShooterDistanceSensorID);
  DigitalInput limitSwitch = new DigitalInput(Constants.kShooterLimitSwitchID);
  private final PositionVoltage m_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

  /** Creates a new shooter. */
  public Shooter() {
    leftShooterWheel.setNeutralMode(NeutralModeValue.Coast);
    rightShooterWheel.setNeutralMode(NeutralModeValue.Coast);
    rightShooterIntake.setIdleMode(IdleMode.kBrake);
    leftShooterIntake.setIdleMode(IdleMode.kBrake);
    distanceSensor.setAverageBits(4);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 0.7; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kD = 0.1;
    leftWrist.setPosition(0);
    leftWrist.getConfigurator().apply(configs);
    leftWrist.setPosition(0);
  }


  public void setShooterFlywheelSpeed(double speed){
    leftShooterWheel.set(-speed);
    rightShooterWheel.set(speed);
  }

  public void setIntakeSpeed(double speed){
    leftShooterIntake.set(-speed);
    rightShooterIntake.set(
      speed);
  }

  public void setWristAngle(double position){
    leftWrist.setControl(m_voltagePosition.withPosition(position));
  }

  public double shooterAngle(){
    return(leftWrist.getPosition().getValueAsDouble());
  }

  public boolean hasNote(){
    return(distanceSensor.getAverageValue() > Constants.kShooterDistanceSensorTreshold);
  }

  public boolean limitSwitch(){
    return(limitSwitch.get());
  }

  public void setZero(double position){
    leftWrist.setPosition(position);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
