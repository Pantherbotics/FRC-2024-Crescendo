// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class calibrateShooter extends Command {
  /** Creates a new calibrateShooter. */
  private final Shooter shooter;
  private double zero;
  private boolean finished;
  
  public calibrateShooter(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    
    addRequirements(shooter);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    shooter.setWristAngle(0);
    zero = shooter.shooterAngle();
    System.out.println("started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    zero += 0.1;
    shooter.setWristAngle(zero);
    if (shooter.limitSwitch()){
      finished = true;
      shooter.setZero(13);
      System.out.println("ZEROED");
      shooter.setWristAngle(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
