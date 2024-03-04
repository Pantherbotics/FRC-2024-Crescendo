// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climbers. */
  public TalonFX leftClimber = new TalonFX(Constants.kLeftClimberMotorID);
  public TalonFX rightClimber = new TalonFX(Constants.kRightClimberMotorID);
  private final PositionVoltage m_voltagePosition;
  private Pigeon2 gyro = new Pigeon2(13);

  public Climber() {
    leftClimber.setNeutralMode(NeutralModeValue.Brake);
    rightClimber.setNeutralMode(NeutralModeValue.Brake);
    m_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
    leftClimber.setPosition(0);
    rightClimber.setPosition(0);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 1.5;
    configs.Slot0.kD = 0;
    configs.Slot0.kI = 0;

    leftClimber.getConfigurator().apply(configs);
    rightClimber.getConfigurator().apply(configs);
  }

  public void setHeight(double height){
    leftClimber.setControl(m_voltagePosition.withPosition(height));
    rightClimber.setControl(m_voltagePosition.withPosition(-height));
  }

  public void setIndividualHeights(double leftHeight, double rightHeight){
    leftClimber.setControl(m_voltagePosition.withPosition(leftHeight));
    rightClimber.setControl(m_voltagePosition.withPosition(rightHeight));
  }

  public void changeIndividualHeights(double leftChange, double rightChange){
    setIndividualHeights(leftClimber.getPosition().getValueAsDouble() + leftChange, rightClimber.getPosition().getValueAsDouble() + rightChange);
  }

  public void setIndividualSpeeds(double leftSpeed, double rightSpeed){
    leftClimber.set(leftSpeed);
    rightClimber.set(rightSpeed);
  }

  public Rotation3d getGyroRotation3d(){
    return gyro.getRotation3d();
  }

  public double getLeftHeight(){
    return leftClimber.getPosition().getValueAsDouble();
  }
  public double getRightHeight(){
    return rightClimber.getPosition().getValueAsDouble();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("left climber", getLeftHeight());
    SmartDashboard.putNumber("right climer", getRightHeight());
    // This method will be called once per scheduler ru
    }
  
}
