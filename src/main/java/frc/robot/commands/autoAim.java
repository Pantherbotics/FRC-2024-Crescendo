// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class autoAim extends Command {
  /** Creates a new runShooter. */
  private final Shooter shooter;
  private final CommandSwerveDrivetrain swerve;
  private double shooterAngle;
  private SwerveRequest.FieldCentricFacingAngle facing;
  private Pose2d robotPose;
  private CommandXboxController joystick;
  private Rotation2d rotationToGoal;

  public autoAim(Shooter shooter, CommandSwerveDrivetrain swerve, SwerveRequest.FieldCentricFacingAngle facing, CommandXboxController joystick) {
    this.joystick = joystick;
    this.shooter = shooter;
    this.swerve = swerve;
    this.facing = facing;
    addRequirements(shooter, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    robotPose = swerve.getState().Pose;
    shooterAngle = shooter.radiansToWristAngle(Math.atan(Constants.kSpeakerHeight/robotPose.getTranslation().getDistance(Constants.kSpeakerPose.getTranslation())));
    rotationToGoal = new Rotation2d(robotPose.getX() - Constants.kSpeakerPose.getX(), robotPose.getY() - Constants.kSpeakerPose.getY());
    
    System.out.println(rotationToGoal);

    CommandScheduler.getInstance().schedule(swerve.applyRequest(() ->
     facing.withVelocityX(-joystick.getLeftY() * 6) // Drive forward with negative Y (forward)
          .withVelocityY(-joystick.getLeftX() * 6) // Drive left with negative X (left)
          .withTargetDirection(rotationToGoal)
    )

    );
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("ending" + (interrupted ? "true" : "false"));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return joystick.rightBumper().getAsBoolean();
  }
}
