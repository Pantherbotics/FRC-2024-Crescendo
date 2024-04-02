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
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {

  TalonFX rightShooterWheel = new TalonFX(Constants.kRightShooterID);
  TalonFX leftShooterWheel = new TalonFX(Constants.kLeftShooterID);

  public TalonFX wrist = new TalonFX(Constants.kwristID);

  CANSparkMax rightShooterIntake = new CANSparkMax(Constants.kRightShooterIntakeID, MotorType.kBrushless);
  CANSparkMax leftShooterIntake = new CANSparkMax(Constants.kLeftShooterIntakeID, MotorType.kBrushless);

  DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.kShooterEncoderID);
  AnalogInput sideSensor = new AnalogInput(Constants.kShooterSideSensorID);
  AnalogInput topSensor = new AnalogInput(Constants.kShooterTopSensorID);
  double lastSpeed;
  double lastTime;
  ProfiledPIDController controller;
  SimpleMotorFeedforward feedforward;
  boolean openLoop = false;

  /** Creates a new shooter. */
  public Shooter() {
    encoder.setDistancePerRotation(Constants.kShooterRatio);

    leftShooterWheel.setNeutralMode(NeutralModeValue.Brake);
    rightShooterWheel.setNeutralMode(NeutralModeValue.Brake);
    rightShooterIntake.setIdleMode(IdleMode.kBrake);
    leftShooterIntake.setIdleMode(IdleMode.kBrake);
    sideSensor.setAverageBits(4);
    topSensor.setAverageBits(4);
    leftShooterWheel.set(0);
    rightShooterWheel.set(0);
    leftShooterIntake.set(0);
    rightShooterIntake.set(0);

    lastSpeed = 0;
    lastTime = Timer.getFPGATimestamp();

    feedforward = new SimpleMotorFeedforward(0, 0, 0);

    this.controller = new ProfiledPIDController(
      0.65, 0, 0.001,
      new TrapezoidProfile.Constraints(13, 30)
    );
    
    this.controller.setGoal(0);
    this.controller.setTolerance(0.3);
  }


  public void setShooterFlywheelSpeed(double speed){
    leftShooterWheel.set(-speed*0.3);
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
    wrist.set(speed);
  }

  public void setOpenLoop(boolean open){
    openLoop = open;
  }

  public boolean isAtGoal(){
    return controller.atGoal();
  }

  public double radiansToWristAngle(double radians){
    return Units.radiansToRotations(radians) * Constants.kShooterRatio;
  }

  public double shooterAngle(){
    return(wrist.getPosition().getValueAsDouble());
  }

  public boolean hasNote(){
    return(sideSensor.getAverageValue() > Constants.kShooterSideSensorTreshold);
  }

  public boolean noteInPosition(){
    return(topSensor.getAverageValue() < 2100 && topSensor.getAverageValue() > 1900);
  }


  public void setZero(double position){
    wrist.setPosition(position);
  }

  public void resetEncoder(){
    encoder.reset();
  }
  

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("top sensor", topSensor.getAverageValue());
    SmartDashboard.putBoolean("top note", noteInPosition());
    SmartDashboard.putNumber("side sensor", sideSensor.getAverageValue());
    SmartDashboard.putBoolean("Shooter Note", hasNote());
    SmartDashboard.putNumber("wrist", shooterAngle());
    SmartDashboard.putBoolean("encoder Connected", encoder.isConnected());
    SmartDashboard.putNumber("distance", encoder.getDistance() + Constants.kShooterEncoderOffset);


    double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    
    if (!openLoop) {
      wrist.setVoltage(
          controller.calculate(-(encoder.getDistance() + Constants.kShooterEncoderOffset))
          + feedforward.calculate(controller.getSetpoint().velocity, acceleration));
    }

    lastSpeed = controller.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
    
  }
}
