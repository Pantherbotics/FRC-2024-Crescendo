// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class autoAimNoMove extends Command {
  private final Shooter shooter;
  private final CommandSwerveDrivetrain swerve;
  private final SwerveRequest.FieldCentricFacingAngle facing;
  private double shooterAngle;
  private Pose2d robotPose;
  private Rotation2d rotationToGoal;

  /** Creates a new autoAimShooterOnly. */
  public autoAimNoMove(Shooter shooter, CommandSwerveDrivetrain swerve, SwerveRequest.FieldCentricFacingAngle facing) {
    this.shooter = shooter;
    this.swerve = swerve;
    this.facing = facing;
    addRequirements(shooter, swerve);
  }

  @Override
  public void initialize() {
    shooter.setShooterFlywheelSpeed(Constants.kShooterSpinSpeed);
  }

  @Override
  public void execute() {
    robotPose = swerve.getState().Pose;
    shooterAngle = shooter.radiansToWristAngle(Math.atan((Constants.kSpeakerHeight - Constants.kShooterHeight )/robotPose.getTranslation().getDistance(Constants.kSpeakerPose.getTranslation())));
    rotationToGoal = new Rotation2d(robotPose.getX() - Constants.kSpeakerPose.getX(), robotPose.getY() - Constants.kSpeakerPose.getY());
    
    shooter.setWristAngle(shooterAngle);
    
    swerve.setControl(
      facing.withTargetDirection(rotationToGoal)
    );


  }

  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().schedule(
      new SequentialCommandGroup(
        new setShooterIntakeSpeed(shooter, -1),
        new WaitUntilCommand(()->!shooter.hasNote()),
        new WaitCommand(0.75),
        new setShooterIntakeSpeed(shooter, 0),
        new setShooterSpeed(shooter, Constants.kShooterIdleSpeed),
        new setShooterAngle(shooter, Constants.kShooterHandoffPosition)
      )
    );
  }

  @Override
  public boolean isFinished() {
    return shooter.isAtGoal() && shooter.wrist.getAcceleration().getValueAsDouble() < 0.1 && Math.abs(swerve.getState().Pose.getRotation().minus(rotationToGoal).getRotations()) < 0.01; // waits for shooter to be at correct angle and the robot to be at the correct rotation
  }
}
