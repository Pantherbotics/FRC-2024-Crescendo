// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

public class autoTargetNote extends Command {
  /** Creates a new autoTargetNote. */
  private CommandSwerveDrivetrain drivetrain;
  private Vision vision;
  private Intake intake;
  public autoTargetNote(CommandSwerveDrivetrain drivetrain, Vision vision, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.intake = intake;
    addRequirements(drivetrain, vision, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
