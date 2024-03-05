// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  TalonFX rightShooterWheel = new TalonFX(Constants.kRightShooterID);
  TalonFX leftShooterWheel = new TalonFX(Constants.kLeftShooterID);

  TalonFX leftWrist = new TalonFX(Constants.kLeftWristID);

  CANSparkMax rightShooterIntake = new CANSparkMax(Constants.kRightShooterIntakeID, MotorType.kBrushless);
  CANSparkMax leftShooterIntake = new CANSparkMax(Constants.kLeftShooterIntakeID, MotorType.kBrushless);

  DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.kShooterEncoderID);
  AnalogInput mydistanceSensor = new AnalogInput(Constants.kShooterDistanceSensorID);
  DigitalInput limitSwitch = new DigitalInput(Constants.kShooterLimitSwitchID);
  double lastSpeed;
  double lastTime;
  ProfiledPIDController controller;
  SimpleMotorFeedforward feedforward;
  int sensorValue = 0;
  boolean openLoop = false;

  /** Creates a new shooter. */
  public Shooter() {
    encoder.setDistancePerRotation(Constants.kShooterRatio);

    leftShooterWheel.setNeutralMode(NeutralModeValue.Brake);
    rightShooterWheel.setNeutralMode(NeutralModeValue.Brake);
    rightShooterIntake.setIdleMode(IdleMode.kBrake);
    leftShooterIntake.setIdleMode(IdleMode.kBrake);
    mydistanceSensor.setAverageBits(4);
    leftShooterWheel.set(0);
    rightShooterWheel.set(0);
    leftShooterIntake.set(0);
    rightShooterIntake.set(0);

    lastSpeed = 0;
    lastTime = Timer.getFPGATimestamp();

    feedforward = new SimpleMotorFeedforward(0, 0, 0);

    this.controller = new ProfiledPIDController(
      0.54, 0, 0.001,
      new TrapezoidProfile.Constraints(10, 25)
    );
    
    if (encoder.isConnected()){
      leftWrist.setPosition(0);
      //leftWrist.setPosition(encoder.getAbsolutePosition() + Constants.kShooterEncoderOffset);
    } else {
      System.out.println("ENCODER NOT CONNETED");
      leftWrist.setPosition(0);
    }
    this.controller.setGoal(0);
    this.controller.setTolerance(0.5);
  }


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
    openLoop = false;
    controller.setGoal(position);
  }

  public void setWristOpenLoop(double speed){
    openLoop = true;
    leftWrist.set(speed);
  }

  public void setOpenLoop(boolean open){
    openLoop = open;
  }

  public boolean isAtGoal(){
    return controller.atGoal();
  }

  public double radiansToWristAngle(double radians){
    System.out.println(Units.radiansToRotations(radians) * (1/ 25) * (12/37));
    return Units.radiansToRotations(radians) * (1/ 25) * (12/37);
  }

  public double shooterAngle(){
    return(leftWrist.getPosition().getValueAsDouble());
  }

  public boolean hasNote(){
    return(sensorValue > Constants.kShooterDistanceSensorTreshold);
  }

  public boolean limitSwitch(){
    return(limitSwitch.get());
  }

  public void setZero(double position){
    leftWrist.setPosition(position);
  }
  

  @Override
  public void periodic() {
    
    sensorValue = mydistanceSensor.getAverageValue();
    SmartDashboard.putBoolean("shooter switch", limitSwitch());
    SmartDashboard.putBoolean("direct switch", limitSwitch.get());
    SmartDashboard.putNumber("shooter distance sensor", sensorValue);
    SmartDashboard.putBoolean("Shooter Note", hasNote());
    SmartDashboard.putNumber("wrist", shooterAngle());
    SmartDashboard.putBoolean("encoder Connected", encoder.isConnected());
    SmartDashboard.putNumber("encoder value", encoder.get());
    SmartDashboard.putNumber("distance", encoder.getDistance());


    double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    
    if (!openLoop) {
      leftWrist.setVoltage(
          controller.calculate(shooterAngle())
          + feedforward.calculate(controller.getSetpoint().velocity, acceleration));
    }

    lastSpeed = controller.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
    
  }
}
