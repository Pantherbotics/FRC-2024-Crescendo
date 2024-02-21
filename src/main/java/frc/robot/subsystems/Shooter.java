// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.MutableMeasure.mutable;




public class Shooter extends SubsystemBase {

  TalonFX rightShooterWheel = new TalonFX(Constants.kRightShooterID);
  TalonFX leftShooterWheel = new TalonFX(Constants.kLeftShooterID);

  TalonFX leftWrist = new TalonFX(Constants.kLeftWristID);

  CANSparkMax rightShooterIntake = new CANSparkMax(Constants.kRightShooterIntakeID, MotorType.kBrushless);
  CANSparkMax leftShooterIntake = new CANSparkMax(Constants.kLeftShooterIntakeID, MotorType.kBrushless);
  
  AnalogInput distanceSensor = new AnalogInput(Constants.kShooterDistanceSensorID);
  DigitalInput limitSwitch = new DigitalInput(Constants.kShooterLimitSwitchID);

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));
  //this way seems more convoluted than just reallocating, but its in the wpilib examples.

  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                leftWrist.setVoltage(volts.magnitude());
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-wheel")
                    .voltage(
                        appliedVoltage.mut_replace(
                            leftWrist.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(angle.mut_replace(leftWrist.getPosition().getValueAsDouble(), Rotations))
                    .angularVelocity(
                        velocity.mut_replace(leftWrist.getVelocity().getValueAsDouble(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));
  
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

   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

   public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}