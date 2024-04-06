// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class reverseShoot extends Command {
  /** Creates a new reverseShoot. */

    private final Shooter shooter;
  private final CommandSwerveDrivetrain swerve;
  private double shooterAngle;
  private SwerveRequest.FieldCentric fieldCentric;
  private Pose2d robotPose;
  private CommandXboxController joystick;
  private Rotation2d rotationToGoal;
  Boolean readyToShoot = false;
  Boolean finished = false;
  boolean reverseShot;


  public reverseShoot(Shooter shooter, CommandSwerveDrivetrain swerve, SwerveRequest.FieldCentric fieldCentric, boolean reverseShot) {
    this.shooter = shooter;

    this.reverseShot = reverseShot; 
    this.swerve = swerve;
    this.fieldCentric = fieldCentric;
    addRequirements(shooter, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    readyToShoot = false;
    shooter.setShooterFlywheelSpeed(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = swerve.getState().Pose;
    Pose2d shooterPose = robotPose.plus(new Transform2d(Units.inchesToMeters(13), 0.0, new Rotation2d(0)));
    double shooterDistance = Math.hypot(shooterPose.getX() - Constants.kSpeakerPose.getX(), shooterPose.getY() - Constants.kSpeakerPose.getY());

    shooterAngle = shooter.radiansToWristAngle(new Rotation2d(Units.inchesToMeters(59 + shooterDistance * 7.6), shooterDistance).getRadians());

    rotationToGoal = new Rotation2d(robotPose.getX() - Constants.kSpeakerPose.plus(new Transform2d(0, -0.1, new Rotation2d())).getX(), robotPose.getY() - Constants.kSpeakerPose.plus(new Transform2d(0, -0.1, new Rotation2d())).getY()).plus(Rotation2d.fromDegrees(180));

    swerve.setControl(   
      fieldCentric
          .withRotationalRate(Math.min(rotationToGoal.minus(swerve.getState().Pose.getRotation().plus(Rotation2d.fromDegrees(180))).getRadians()*3, 3))
    );

    shooter.setWristAngle(shooterAngle);

    readyToShoot = rotationToGoal.getDegrees() < 5 && shooter.isAtGoal() && shooter.wrist.getAcceleration().getValueAsDouble() < 0.5;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
