// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  
  SparkFlex shooter1;
  SparkFlex shooter2;
  
  SparkFlexConfig shooter1Config;
  SparkFlexConfig shooter2Config;

  RelativeEncoder shooter1Encoder;
  RelativeEncoder shooter2Encoder;
  

  public Shooter() {
    shooter1 = new SparkFlex(6, MotorType.kBrushless);
    shooter2 = new SparkFlex(7, MotorType.kBrushless);


    shooter1Encoder = shooter1.getEncoder();
    shooter2Encoder = shooter2.getEncoder();

    shooter1Config = new SparkFlexConfig();
    shooter2Config = new SparkFlexConfig();

    shooter1Config.smartCurrentLimit(75);
    shooter1Config.openLoopRampRate(.1);
    shooter1Config.closedLoopRampRate(.1);

    shooter2Config.follow(shooter1);

    shooter1.configure(shooter1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    shooter2.configure(shooter2Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
