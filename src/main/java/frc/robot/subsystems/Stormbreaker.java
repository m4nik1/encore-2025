// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Stormbreaker extends SubsystemBase {
  /** Creates a new Stormbreaker. */

  SparkMax storm;
  SparkMax breaker;

  SparkClosedLoopController stormPID;

  RelativeEncoder stormCoder;
  RelativeEncoder stormAlternateCoder;

  SparkMaxConfig stormConfig;
  SparkMaxConfig breakerConfig;

  AnalogPotentiometer pot;

  DigitalInput armLimit;

  

  // shooters will be 6 and 7 vortexs follower
  // DIO 1 for limit switch
  // DIO 0 for beam break sensor
  
  public Stormbreaker() {
    storm = new SparkMax(5, MotorType.kBrushless);
    breaker = new SparkMax(11, MotorType.kBrushless);

    armLimit = new DigitalInput(1);


    stormConfig = new SparkMaxConfig();
    breakerConfig = new SparkMaxConfig();

    stormConfig.encoder.positionConversionFactor(74.4827586207).velocityConversionFactor(74.4827586207);

    stormCoder = storm.getEncoder();
    stormAlternateCoder = storm.getAlternateEncoder();

    pot = new AnalogPotentiometer(0);
    stormPID = storm.getClosedLoopController();

    breakerConfig.follow(storm, true);

    stormConfig.closedLoop
                  .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder).pidf(.062, 0, 0, .002);

    stormConfig.alternateEncoder.apply(new AlternateEncoderConfig().setSparkMaxDataPortConfig());

    stormConfig.idleMode(IdleMode.kBrake);


    stormCoder.setPosition(0);
    stormAlternateCoder.setPosition(0);

    stormConfig.closedLoopRampRate(.1);

    storm.configure(stormConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    breaker.configure(breakerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getPot() {
    return pot.get();
  }

  public boolean getarmLimit() {
    return armLimit.get();
  }

  public double getStormPos() {
    return stormAlternateCoder.getPosition();
  }

  public void summonStormBreaker(double pos) {
    stormPID.setReference(pos, ControlType.kPosition);
  }

  public void speakerPos() {
    stormPID.setReference(10, ControlType.kPosition);
  }

  public double getStormPwr() {
    return storm.getAppliedOutput();
  }

  public double getBreakerPwr() {
    return breaker.getAppliedOutput();
  }

  public void swingStormBreaker(double spd) {
    storm.set(spd*0.47); // was .47
  }

  public boolean atArmSpeaker() {
    return getStormPos() > 9;
  }

  public boolean atArmGround() {
    return getStormPos() < 0.7;
  }
  
  public boolean atArmStow() {
    return getStormPos() > Constants.groundPosition-5;
  }

  // public void stormBreakerBrakeMode() {
  //   storm.setIdleMode(IdleMode.kBrake);
  //   breaker.setIdleMode(IdleMode.kBrake);
  // }

  // public void stormBreakerCoast() {
  //   storm.setIdleMode(IdleMode.kCoast);
  //   breaker.setIdleMode(IdleMode.kCoast);
  // }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double stormBreakerPos = getStormPos();

    // SmartDashboard.putNumber("Storm coder", stormCoder.getPosition());
    SmartDashboard.putNumber("Storm Breaker Position", stormBreakerPos);
    // SmartDashboard.putNumber("pot", getPot());
    SmartDashboard.putBoolean("Arm Limit", getarmLimit());

    if(getarmLimit()) {
      stormCoder.setPosition(0);
      stormAlternateCoder.setPosition(0);
    }
  }
}
