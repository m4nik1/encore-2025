// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  Timer timer = new Timer();
  SparkMax intakeMotor; // 8
  DigitalInput noteDetector;

  public Intake() 
  {
    // intakeMotor.restoreFactoryDefaults();
    // intakeMotor = new SparkMax(8, );
    // noteDetector = new DigitalInput(0);
  }

  public void actuateIntake(double power) {
    intakeMotor.set(power);
  }

  public boolean detectNote() {
    return !noteDetector.get();
  }

  // public boolean detectNoteAuto() {
  //   return noteDetector.get();
  // }

  // public boolean detectNewNote() {
  //   return !newNoteDetector.get();
  // }

  public void runIndexer() {
    intakeMotor.set(-.5);
  }

  public void operatorControl(double spd) {
    intakeMotor.set(spd * .65);
  }

  @Override
  public void periodic() {
    timer.start();
  }
}
