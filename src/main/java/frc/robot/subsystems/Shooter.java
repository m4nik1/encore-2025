// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  
  SparkFlex shooter1;
  SparkFlex shooter2;

  RelativeEncoder shooter1Encoder;
  RelativeEncoder shooter2Encoder;
  

  public Shooter() {
    shooter1 = new SparkFlex(6, MotorType.kBrushless);
    shooter2 = new SparkFlex(7, MotorType.kBrushless);


    shooter1Encoder = shooter1.getEncoder();
    shooter2Encoder = shooter2.getEncoder();

    shooter2.follow(shooter1);

    shooter1.setSmartCurrentLimit(75);
    shooter1.setOpenLoopRampRate(.1);
    shooter1.setClosedLoopRampRate(.1);

    shooter1.burnFlash();
    shooter2.burnFlash();
  }

  public void runShooterTest() {
    shooter1.set(-.3);
  }

  public void startShooter() {
    shooter1.set(-.65);
  }

  public boolean isAtSpeed(double rpm) {
    return (shooter1Encoder.getVelocity()) < -rpm && (shooter2Encoder.getVelocity()) < -rpm;
  }

    public void stopShooter() {
    shooter1.set(0);
  }
  
  // public double getShooter() {
  //   return shooter1.getPos`();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double shooterVel = shooter1Encoder.getVelocity();
  }
}
