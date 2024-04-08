// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private PIDController pid  = new PIDController(.1, 0, 0.006);
  private boolean collectedNote;

  public autoTargetNote(CommandSwerveDrivetrain drivetrain, Intake intake, Shooter shooter, SwerveRequest.RobotCentric robotCentric ) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.robotCentric = robotCentric;
    this.shooter = shooter;
    addRequirements(drivetrain, intake, shooter);
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    collectedNote = false;
    intake.setAngle(Constants.kIntakeDownPosition);
    intake.setSpeed(Constants.kIntakeInSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Vision.noteA > 0.001 && !collectedNote){
      drivetrain.setControl(   
        robotCentric.withVelocityX(1 - Vision.noteX/20 + Vision.noteY/20) // Drive forward with negative Y (forward)
            .withRotationalRate(pid.calculate(Vision.noteX, 0))            
            .withRotationalDeadband(0)
      );
    }

    if (Vision.noteY < 0 && Math.abs(Vision.noteX) < 3 && !collectedNote) {
      collectedNote = true;
      drivetrain.setControl(
        robotCentric.withVelocityX(0.5)
              .withRotationalRate(0)            
              .withRotationalDeadband(0)
      );
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      CommandScheduler.getInstance().schedule(
        new SequentialCommandGroup(
          new WaitCommand(0.1),
          new setIntakeSpeed(intake, 0),
          new intakeHandoff(shooter, intake)
        )
      );
    }
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.hasNote() || Vision.noteA < 0.01;
  }
}
