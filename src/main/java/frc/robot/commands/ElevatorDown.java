// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

public class ElevatorDown extends Command {
  /** Creates a new ElevatorDown. */
  private final Elevator m_elevator;
  private final Shooter m_shooter;

  public ElevatorDown(Elevator elevator, Shooter shooter) {
    m_elevator = elevator;
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.handoffPosition();
    m_shooter.handoffPosition();
  }

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
