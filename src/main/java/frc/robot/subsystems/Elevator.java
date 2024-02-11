// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  TalonFX elevatorMotor = new TalonFX(Constants.kElevatorMotorID);
  CANcoder elevatorEncoder = new CANcoder(Constants.kElevatorEncoderID);

  private final PositionVoltage m_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

  public Elevator() {

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4;
    configs.Slot0.kD = 0.1;
    elevatorMotor.getConfigurator().apply(configs);
    elevatorMotor.setPosition(elevatorEncoder.getAbsolutePosition().getValueAsDouble() + Constants.kElevatorEncoderOffset);
  }

  public void handoffPosition(){
    elevatorMotor.setControl(m_voltagePosition.withPosition(Constants.kElevatorIntakePosition));
  }
  public void ampPosition(){
    elevatorMotor.setControl(m_voltagePosition.withPosition(Constants.kElevatorAmpPosition));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
