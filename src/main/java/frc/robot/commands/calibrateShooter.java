// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class calibrateShooter extends Command {
  /** Creates a new calibrateShooter. */
  private final Shooter shooter;
  
  public calibrateShooter(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    
    addRequirements(shooter);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setWristOpenLoop(0.175);
    System.out.println("started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setZero(9.25);
    System.out.println("ZEROED");
    shooter.setWristAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.limitSwitch();
  }
}
