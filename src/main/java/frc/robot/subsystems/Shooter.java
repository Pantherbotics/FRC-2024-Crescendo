// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
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

  /** Creates a new shooter. */
  public Shooter() {
    leftShooterWheel.setNeutralMode(NeutralModeValue.Coast);
    rightShooterWheel.setNeutralMode(NeutralModeValue.Coast);
    rightShooterIntake.setIdleMode(IdleMode.kBrake);
    leftShooterIntake.setIdleMode(IdleMode.kBrake);
    distanceSensor.setAverageBits(4);

    var wristConfigs = new TalonFXConfiguration();
    var slot0Configs = wristConfigs.Slot0;
    
    //examples taken from ctre docs
    slot0Configs.kS = 0.25; //replace all these values with https://www.reca.lc/arm
    slot0Configs.kV = 0.12; //and later we can use the SysID routines
    slot0Configs.kA = 0.01; 

    slot0Configs.kP = 1; 
    slot0Configs.kI = 0; 
    slot0Configs.kD = 0; 

    // examples taken from ctre docs
    var motionMagicConfigs = wristConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; 
    motionMagicConfigs.MotionMagicAcceleration   = 160; 
    motionMagicConfigs.MotionMagicJerk           = 1600; 

    leftWrist.getConfigurator().apply(wristConfigs);
  };


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
    leftWrist.setControl(new MotionMagicVoltage(position));
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
}