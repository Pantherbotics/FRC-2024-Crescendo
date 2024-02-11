// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  /** Creates a new intake. */
  TalonFX intakeRoller = new TalonFX(Constants.kIntakeRollerID);
  TalonFX intakePivot = new TalonFX(Constants.kIntakePivotID);
  CANcoder intakeEncoder = new CANcoder(Constants.kIntakeEncoderID);
  AnalogInput distanceSensor = new AnalogInput(Constants.kIntakeDistanceSensorID);

  private final PositionVoltage m_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);


  public Intake() {
    distanceSensor.setAverageBits(4);

    //pivot PID
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4;
    configs.Slot0.kD = 0.1;
    intakePivot.getConfigurator().apply(configs);
    intakePivot.setPosition(intakeEncoder.getAbsolutePosition().getValueAsDouble() + Constants.kIntakeEncoderOffset);
  }


  public void intakeDown(){
    intakePivot.setControl(m_voltagePosition.withPosition(Constants.kIntakeDownPosition));
  }

  public void intakeUp(){
    intakePivot.setControl(m_voltagePosition.withPosition(Constants.kIntakeUpPosition));
  }

  public void intakeIn(){
    intakeRoller.set(Constants.kIntakeInSpeed);
  }

  public void intakeStop(){
    intakeRoller.set(0);
  }

  public void intakeOut(){
    intakeRoller.set(Constants.kIntakeHandoffSpeed);
  }

  public Boolean hasNote(){
    return(distanceSensor.getAverageValue() > Constants.kIntakeDistanceSensorThreshold);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
