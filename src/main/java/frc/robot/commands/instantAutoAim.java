// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class instantAutoAim extends InstantCommand {
  private final Shooter shooter;
  private final CommandSwerveDrivetrain swerve;
  private double shooterAngle;
  private SwerveRequest.FieldCentricFacingAngle facing;
  private Pose2d robotPose;
  private CommandXboxController joystick;
  private Rotation2d rotationToGoal;

  public instantAutoAim(Shooter shooter, CommandSwerveDrivetrain swerve, SwerveRequest.FieldCentricFacingAngle facing, CommandXboxController joystick) {
    this.shooter = shooter;
    this.swerve = swerve;
    this.facing = facing;
    this.joystick = joystick;
    addRequirements(shooter, swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    robotPose = swerve.getState().Pose;
    shooterAngle = shooter.radiansToWristAngle(Math.atan((Constants.kSpeakerHeight - Constants.kShooterHeight )/robotPose.getTranslation().getDistance(Constants.kSpeakerPose.getTranslation())));
    //rotationToGoal = new Rotation2d(robotPose.getX() - Constants.kSpeakerPose.getX(), robotPose.getY() - Constants.kSpeakerPose.getY());
    
    //System.out.println(rotationToGoal);

    /*CommandScheduler.getInstance().schedule(
      new ParallelCommandGroup(
        swerve.applyRequest(() ->
        facing.withVelocityX(-joystick.getLeftY() * 6) // Drive forward with negative Y (forward)
          .withVelocityY(-joystick.getLeftX() * 6) // Drive left with negative X (left)
          .withTargetDirection(rotationToGoal)
        ),
        new setShooterAngle(shooter, shooterAngle)
      )

    );
    */

    shooter.setWristAngle(shooterAngle);

  }
}
