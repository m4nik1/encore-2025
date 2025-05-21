// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  public static Intake intake = new Intake();
  public static Shooter shooter = new Shooter();
  public static DriveTrainKrakens driveTrain = new DriveTrainKrakens();
  public static Stormbreaker stormBreaker = new Stormbreaker();

  static CommandXboxController driver = new CommandXboxController(0);
  static CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    driveTrain.setDefaultCommand(new TelopDrive());
    stormBreaker.setDefaultCommand(new swingStormBreaker());
    configureBindings();
  }

  private void configureBindings() {
    operator.rightTrigger().whileTrue(new IntakeIn());
    operator.leftTrigger().whileTrue(new revShooter());

    driver.rightBumper().onTrue(new InstantCommand(() -> driveTrain.resetGyro()));

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

  public static double getLeftYOp() {
    return -operator.getLeftY();
  }

  public static double getRightYOp() {
    return operator.getRightY();
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
