// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class revShooter extends Command {
  /** Creates a new revShooter. */
  public revShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter.startShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
