// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  public static Intake intake = new Intake();
  public static Shooter shooter = new Shooter();
  public static DriveTrainKrakens driveTrain = new DriveTrainKrakens();

  static CommandXboxController driver = new CommandXboxController(0);
  static CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    driveTrain.setDefaultCommand(new TelopDrive());
    configureBindings();
  }

  private void configureBindings() {

  }

  public static double getLeftX() {
    return driver.getLeftX();
  }

  public static double getRightX() {
    return driver.getRightX();
  }

  public static double getLeftY() {
    return driver.getLeftY();
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
