// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
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
  double lastSpeed;
  double lastTime;
  ProfiledPIDController controller;
  ArmFeedforward feedforward;


  public Intake() {
    distanceSensor.setAverageBits(4);
    
    intakeRoller.setNeutralMode(NeutralModeValue.Brake);
    intakePivot.setNeutralMode(NeutralModeValue.Brake);
    intakeRoller.set(0);
    intakePivot.set(0);
    intakePivot.setPosition(0);
    

    lastSpeed = 0;
    lastTime = Timer.getFPGATimestamp();

    feedforward = new ArmFeedforward(0, 0, 0);

    this.controller = new ProfiledPIDController(
      0.5, 0, 0.1,
      new TrapezoidProfile.Constraints(6, 2)
    );

    this.controller.setGoal(0);
    this.controller.setTolerance(0.3);

  }

  public void setAngle(double goalPosition) {
    controller.setGoal(goalPosition);
  } 

  public Boolean isAtGoal(){
    return controller.atGoal();
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
    double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    intakePivot.setVoltage(
        controller.calculate(intakeAngle())
        + feedforward.calculate(controller.getSetpoint().velocity, acceleration));
    lastSpeed = controller.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
  }

}
