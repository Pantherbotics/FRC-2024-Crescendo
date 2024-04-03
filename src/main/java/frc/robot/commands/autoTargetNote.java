// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class autoTargetNote extends Command {
  /** Creates a new autoTargetNote. */
  private CommandSwerveDrivetrain drivetrain;
  private Intake intake;
  private Shooter shooter;
  private SwerveRequest.RobotCentric robotCentric;
  private boolean handoffAtEnd;

  public autoTargetNote(CommandSwerveDrivetrain drivetrain, Intake intake, Shooter shooter, SwerveRequest.RobotCentric robotCentric, boolean handoffAtEnd ) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.robotCentric = robotCentric;
    this.shooter = shooter;
    this.handoffAtEnd = handoffAtEnd;
    addRequirements(drivetrain, intake, shooter);
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

    if (intake.intakeAngle() > Constants.kIntakeDownPosition - 5){
      drivetrain.setControl(   
        robotCentric.withVelocityX(Math.min((Vision.noteY+7)/10, 1)) // Drive forward with negative Y (forward)
            .withRotationalRate(-Vision.noteX/10)
            .withRotationalDeadband(0)
      );
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSpeed(0);
    if(!interrupted && handoffAtEnd){
      CommandScheduler.getInstance().schedule(new intakeHandoff(shooter, intake));
    }
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.hasNote();
  }
}
