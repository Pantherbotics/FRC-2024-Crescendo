// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

import frc.robot.subsystems.Intake;

public class FullIntake extends SequentialCommandGroup {
  /** Creates a new RunShooter. */
  public FullIntake(Shooter shooter, Intake intake) {
    addCommands(
      new ParallelCommandGroup( new IntakeUntilNote(intake))
      //new ParallelCommandGroup()
      

      //intake run
      //new SequentialCommandGroup(new intake.setAngle(Constants.), new shooter.setAngle(Constants.kIntakeAngle));
    );
  }
}
