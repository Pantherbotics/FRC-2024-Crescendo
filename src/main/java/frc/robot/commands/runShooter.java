// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class runShooter extends Command {
  /** Creates a new runShooter. */
  private final Shooter shooter;
  private final CommandSwerveDrivetrain swerve;
  private double shooterAngle;
  private SwerveRequest.FieldCentricFacingAngle facing;
  private Pose2d robotPose;
  private CommandXboxController joystick;
  private Rotation2d rotationToGoal;
  private boolean manual;

  public runShooter(Shooter shooter, CommandSwerveDrivetrain swerve, SwerveRequest.FieldCentricFacingAngle facing, CommandXboxController joystick, boolean manual) {
    this.joystick = joystick;
    this.shooter = shooter;
    this.swerve = swerve;
    this.facing = facing;
    this.manual = manual;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterFlywheelSpeed(1);
    shooter.setIntakeSpeed(0);
    shooter.setWristAngle(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(manual){
      robotPose = swerve.getState().Pose;
      shooterAngle = shooter.radiansToWristAngle(Math.atan(2.032/robotPose.getTranslation().getDistance(Constants.kSpeakerPose.getTranslation())));
      rotationToGoal = new Rotation2d(robotPose.getX() - Constants.kSpeakerPose.getX(), robotPose.getY() - Constants.kSpeakerPose.getY());
      swerve.applyRequest(()->facing
              .withTargetDirection(rotationToGoal)
              .withVelocityX(-joystick.getLeftX())
              .withVelocityY(-joystick.getLeftY()));
    }
    shooter.setWristAngle((joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis()) * 5);

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
