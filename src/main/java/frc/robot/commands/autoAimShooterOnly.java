// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class autoAimShooterOnly extends Command {
  private final Shooter shooter;
  private final CommandSwerveDrivetrain swerve;
  private double shooterAngle;
  private Pose2d robotPose;
  private Rotation2d rotationToGoal;

  /** Creates a new autoAimShooterOnly. */
  public autoAimShooterOnly(Shooter shooter, CommandSwerveDrivetrain swerve) {
    this.shooter = shooter;
    this.swerve = swerve;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    robotPose = swerve.getState().Pose;
    shooterAngle = shooter.radiansToWristAngle(Math.atan((Constants.kSpeakerHeight - Constants.kShooterHeight )/robotPose.getTranslation().getDistance(Constants.kSpeakerPose.getTranslation())));
    rotationToGoal = new Rotation2d(robotPose.getX() - Constants.kSpeakerPose.getX(), robotPose.getY() - Constants.kSpeakerPose.getY());
    
    shooter.setWristAngle(shooterAngle);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
