// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  // need to switch to https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/mechanisms/swerve/utility/PhoenixPIDController.html
  PhoenixPIDController controller = new PhoenixPIDController(0, 0, 0);
  /** Creates a new intake. */
  TalonFX intakeRoller = new TalonFX(Constants.kIntakeRollerID);
  TalonFX intakePivot = new TalonFX(Constants.kIntakePivotID);
  DigitalInput zeroSwitch = new DigitalInput(Constants.kIntakeLimitSwitchID);
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
    configs.Slot0.kD = 0.35;
    configs.Slot0.kI = 0.01;


    intakePivot.getConfigurator().apply(configs);
    intakePivot.setPosition(0);
    intakePivot.setControl(m_voltagePosition.withPosition(0));
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

  public boolean limitSwitch(){
    return zeroSwitch.get();
  }

  public Command setZero(){
    return new RunCommand(()->intakePivot.setPosition(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
