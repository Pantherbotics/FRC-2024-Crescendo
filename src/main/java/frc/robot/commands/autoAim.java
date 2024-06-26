// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class autoAim extends Command {
  /** Creates a new autoAim. */

  private final Shooter shooter;
  private final CommandSwerveDrivetrain swerve;
  private double shooterAngle;
  private SwerveRequest.FieldCentric fieldCentric;
  private Pose2d robotPose;
  private CommandXboxController joystick;
  private Rotation2d rotationToGoal;
  private Trigger shootButton;
  private boolean autoTrigger;
  boolean readyToShoot = false;
  boolean finished = false;
  boolean triggerReady = false;
  public autoAim(Shooter shooter, CommandSwerveDrivetrain swerve, SwerveRequest.FieldCentric fieldCentric, CommandXboxController joystick, Trigger shootButton, boolean autoTrigger) {
    this.shooter = shooter;
    this.autoTrigger = autoTrigger;
    this.shootButton = shootButton;
    this.swerve = swerve;
    this.fieldCentric = fieldCentric;
    this.joystick = joystick;
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
    if (!shootButton.getAsBoolean()){
      readyToShoot = true;
    }
    robotPose = swerve.getState().Pose;

    Pose2d shooterPose = robotPose.plus(new Transform2d(Units.inchesToMeters(13), 0.0, new Rotation2d(0)));

    double shooterDistance = Math.hypot(shooterPose.getX() - Constants.kSpeakerPose.getX(), shooterPose.getY() - Constants.kSpeakerPose.getY());

    shooterAngle = -shooter.radiansToWristAngle(new Rotation2d(Units.inchesToMeters(38 + shooterDistance * 8), shooterDistance).getRadians());

    rotationToGoal = new Rotation2d(robotPose.getX() - Constants.kSpeakerPose.plus(new Transform2d(0, -0.1, new Rotation2d())).getX(), robotPose.getY() - Constants.kSpeakerPose.plus(new Transform2d(0, -0.1, new Rotation2d())).getY());

    Rotation2d rotationDifference = rotationToGoal.minus(swerve.getState().Pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    if (!autoTrigger){
      swerve.setControl(
        fieldCentric.withVelocityX(-joystick.getLeftY() * 6)
            .withVelocityY(-joystick.getLeftX() * 6)
            .withRotationalRate(Math.min(rotationDifference.getRadians()*4, 3))
      );
    } else {
      swerve.setControl(
        fieldCentric.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(Math.min(rotationDifference.getRadians()*4, 3))
      );
    }

    //shooter.setWristAngle(Constants.kShooterSpeakerAngle + shooter.radiansToWristAngle(Units.rotationsToRadians((joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis())/4)));
    shooter.setWristAngle(shooterAngle);

    SmartDashboard.putNumber("speakerDistance", shooterDistance);
    SmartDashboard.putNumber("target rotation", rotationToGoal.getDegrees());
    SmartDashboard.putNumber("shooter target angle", shooterAngle);

    triggerReady = shooter.controller.atSetpoint() && shooter.rightShooterWheel.getVelocity().getValueAsDouble() > 101 && Math.abs(rotationDifference.getDegrees()) < 5; // check this
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (shootButton.getAsBoolean() && readyToShoot) || (autoTrigger && triggerReady);


  }
}
