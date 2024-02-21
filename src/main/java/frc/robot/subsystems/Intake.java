// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {


  /** Creates a new intake. */
  TalonFX intakeRoller = new TalonFX(Constants.kIntakeRollerID);
  TalonFX intakePivot = new TalonFX(Constants.kIntakePivotID);
  DigitalInput zeroSwitch = new DigitalInput(Constants.kIntakeLimitSwitchID);
  //CANcoder intakeEncoder = new CANcoder(Constants.kIntakeEncoderID);
  AnalogInput distanceSensor = new AnalogInput(Constants.kIntakeDistanceSensorID);

  public Intake() {
    distanceSensor.setAverageBits(4);
    
    intakeRoller.setNeutralMode(NeutralModeValue.Brake);

    intakePivot.setNeutralMode(NeutralModeValue.Brake);
    intakePivot.setPosition(0);

    var intakePivotConfigs = new TalonFXConfiguration();
    var slot0Configs = intakePivotConfigs.Slot0;
    
    //honestly, from my reading, using feedforward and motion profiling is a bit overkill
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
}
