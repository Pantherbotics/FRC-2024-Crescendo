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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climbers. */
  public TalonFX leftClimber = new TalonFX(Constants.kLeftClimberMotorID);
  public TalonFX rightClimber = new TalonFX(Constants.kRightClimberMotorID);
  DigitalInput leftSwitch = new DigitalInput(Constants.kLeftClimberSwitchID);
  DigitalInput rightSwitch = new DigitalInput(Constants.kRightClimberSwitchID);
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

  public void setIndividualSpeeds(double leftSpeed, double rightSpeed){
    leftClimber.set(leftSpeed);
    rightClimber.set(rightSpeed);
  }

  public void setSpeeds(double speed){
    rightClimber.set(speed);
    leftClimber.set(-speed);
  }

  public Rotation3d getGyroRotation3d(){
    return gyro.getRotation3d();
  }


  //getters
  public double getLeftHeight(){
    return leftClimber.getPosition().getValueAsDouble();
  }
  public double getRightHeight(){
    return rightClimber.getPosition().getValueAsDouble();
  }
  public boolean getLeftSwitch(){
    return leftSwitch.get();
  }
  public boolean getRightSwitch(){
    return rightSwitch.get();
  }

  // climb both sides until limit switches are triggered
  public Command climbMaxHeight(){
    return
    new ParallelCommandGroup(
      new RunCommand(
        ()->{
          leftClimber.set(-1);
        }
      ).until(()->getLeftSwitch())      
      .finallyDo(()->leftClimber.set(0)),
      new RunCommand(
        ()->{
          rightClimber.set(1);
        }
      ).until(()->getRightSwitch())
      .finallyDo(()->rightClimber.set(0))
    );
  }


  @Override
  public void periodic() {
    // put values to smartDashboard
    SmartDashboard.putNumber("left climber", getLeftHeight());
    SmartDashboard.putNumber("right climer", -getRightHeight());
    SmartDashboard.putBoolean("left swtich", getLeftSwitch());
    SmartDashboard.putBoolean("right swtich", getRightSwitch());
    }
  
}
