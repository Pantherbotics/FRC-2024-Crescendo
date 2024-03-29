// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class autoTargetNote extends Command {
  /** Creates a new autoTargetNote. */
  private CommandSwerveDrivetrain drivetrain;
  private Vision vision;
  private Intake intake;
  private Shooter shooter;
  private SwerveRequest.RobotCentric robotCentric;

  public autoTargetNote(CommandSwerveDrivetrain drivetrain, Vision vision, Intake intake, Shooter shooter, SwerveRequest.RobotCentric robotCentric) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.robotCentric = robotCentric;
    this.shooter = shooter;
    addRequirements(drivetrain, vision, intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setAngle(Constants.kIntakeDownPosition);
    intake.setSpeed(Constants.kIntakeInSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.notePipelineResult.hasTargets()){
      var bestTarget = vision.notePipelineResult.getBestTarget();
      drivetrain.setControl(   
        robotCentric.withVelocityX(Math.max(-bestTarget.getPitch(), -3)) // Drive forward with negative Y (forward)
            .withRotationalRate(bestTarget.getYaw())
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {CommandScheduler.getInstance().schedule(new intakeHandoff(shooter, intake));}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.hasNote() || !vision.notePipelineResult.hasTargets();
  }
}
