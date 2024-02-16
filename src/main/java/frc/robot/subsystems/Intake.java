// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  /** Creates a new intake. */
  TalonFX intakeRoller = new TalonFX(Constants.kIntakeRollerID);
  TalonFX intakePivot = new TalonFX(Constants.kIntakePivotID);
  //CANcoder intakeEncoder = new CANcoder(Constants.kIntakeEncoderID);
  AnalogInput distanceSensor = new AnalogInput(Constants.kIntakeDistanceSensorID);

  private final PositionVoltage m_voltagePosition;


  public Intake() {
    m_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
    distanceSensor.setAverageBits(4);
    intakeRoller.setNeutralMode(NeutralModeValue.Brake);
    intakePivot.setNeutralMode(NeutralModeValue.Brake);

    //pivot PID
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 1.4;
    configs.Slot0.kD = 0.3;
    configs.Slot0.kI = 0.01;

    intakePivot.setPosition(0);
    intakePivot.getConfigurator().apply(configs);
    
  }



  public void setAngle(double angle){
    
    intakePivot.setControl(m_voltagePosition.withPosition(angle));
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
