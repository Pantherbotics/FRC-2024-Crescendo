// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.MutableMeasure.mutable;

public class Intake extends SubsystemBase {  

 // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));
  //this way seems more convoluted than just reallocating, but its in the wpilib examples.


  /** Creates a new intake. */
  TalonFX intakeRoller = new TalonFX(Constants.kIntakeRollerID);
  TalonFX intakePivot = new TalonFX(Constants.kIntakePivotID);
  DigitalInput zeroSwitch = new DigitalInput(Constants.kIntakeLimitSwitchID);
  //CANcoder intakeEncoder = new CANcoder(Constants.kIntakeEncoderID);
  AnalogInput distanceSensor = new AnalogInput(Constants.kIntakeDistanceSensorID);

  private final SysIdRoutine pivotSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> {
                intakePivot.setVoltage(volts.magnitude());
              },
              log -> {
                log.motor("intake-pivot")
                    .voltage(
                        appliedVoltage.mut_replace(
                            intakePivot.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(angle.mut_replace(intakePivot.getPosition().getValueAsDouble(), Rotations))
                    .angularVelocity(
                        velocity.mut_replace(intakePivot.getVelocity().getValueAsDouble(), RotationsPerSecond));
              },
              this));
  
  private final SysIdRoutine rollerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> {
                intakeRoller.setVoltage(volts.magnitude());
              },
              log -> {
                log.motor("intake-roller")
                    .voltage(
                        appliedVoltage.mut_replace(
                            intakeRoller.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(angle.mut_replace(intakeRoller.getPosition().getValueAsDouble(), Rotations))
                    .angularVelocity(
                        velocity.mut_replace(intakeRoller.getVelocity().getValueAsDouble(), RotationsPerSecond));
              },
              this));

  public Intake() {
    distanceSensor.setAverageBits(4);
    
    intakeRoller.setNeutralMode(NeutralModeValue.Brake);

    intakePivot.setNeutralMode(NeutralModeValue.Brake);
    intakePivot.setPosition(0);

    var intakePivotConfigs = new TalonFXConfiguration();
    var slot0Configs = intakePivotConfigs.Slot0;
    
    //honestly, from my reading, using feedforward and motion profiling for this pivot is a bit overkill
    //examples taken from ctre docs
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output

    slot0Configs.kP = 1; 
    slot0Configs.kI = 0; 
    slot0Configs.kD = 0; 

    // examples taken from ctre docs
    var motionMagicConfigs = intakePivotConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    intakePivot.getConfigurator().apply(intakePivotConfigs);
  }

  public void setAngle(double goalPosition) {
    intakePivot.setControl(new MotionMagicVoltage(goalPosition));
  } 

  public void setSpeed(double speed){
    intakeRoller.set(speed);
  }

  public Boolean hasNote(){
    return(distanceSensor.getAverageValue() > Constants.kIntakeDistanceSensorThreshold);
  }

  public double intakeAngle(){
    return(intakePivot.getPosition().getValueAsDouble());
  }

  public boolean limitSwitch(){
    return zeroSwitch.get();
  }

  public Command setZero(){
    return new RunCommand(()->intakePivot.setPosition(0));
  }

  public Command pivotSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return pivotSysIdRoutine.quasistatic(direction);
  }

   public Command pivotSysIdDynamic(SysIdRoutine.Direction direction) {
    return pivotSysIdRoutine.dynamic(direction);
  }

   public Command rollerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return rollerSysIdRoutine.quasistatic(direction);
  }

   public Command rollerSysIdDynamic(SysIdRoutine.Direction direction) {
    return rollerSysIdRoutine.dynamic(direction);
  }
}
