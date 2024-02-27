// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class stabilizedClimb extends Command {
  private final Climber climber;
  private double height;
  private double leftClimberChange;
  private double rightClimberChange;
  private double leftHeight;
  private double rightHeight;
  /** Creates a new setClimberHeight. */
  public stabilizedClimb(Climber climber, double height) {
    this.climber = climber;
    this.height = height;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rightHeight = climber.rightClimber.getPosition().getValueAsDouble();
    leftHeight = climber.leftClimber.getPosition().getValueAsDouble();
    if (leftHeight > Constants.kClimberDownPosition){
      leftClimberChange = 0.5 + Units.radiansToRotations(climber.getGyroRotation3d().getX());
    } else {
      leftClimberChange = 0;
    }
    if (rightHeight > Constants.kClimberDownPosition){
      rightClimberChange = 0.5 - Units.radiansToRotations(climber.getGyroRotation3d().getX());
    } else {
      rightClimberChange = 0;
    }
    climber.setIndividualSpeeds(leftClimberChange, rightClimberChange);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
