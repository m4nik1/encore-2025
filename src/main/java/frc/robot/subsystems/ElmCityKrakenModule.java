// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElmCityKrakenModule extends SubsystemBase {
  /** Creates a new ElmCityKrakenModule. */

  TalonFX driveMotor;
  TalonFX angleMotor;

  CANcoder nacCoder;


  PositionDutyCycle anglePosition = new PositionDutyCycle(0);
  DutyCycleOut driveOpenLoop;
  VelocityVoltage driveVelocity;
  SimpleMotorFeedforward driveKfCalc = new SimpleMotorFeedforward(Constants.driveKs, Constants.driveKv, Constants.drivekA);

  public int modNum;
  Rotation2d lastAngle;
  Rotation2d offset;
  InvertedValue driveInverted;

  public ElmCityKrakenModule(int moduleNumber, int driveID, int angleID, int nacID, Rotation2d givenOffset,
      InvertedValue driveInvert,
      InvertedValue angleInvert) {
    this.modNum = moduleNumber;
    this.offset = givenOffset;
    this.driveInverted = driveInvert;

    driveMotor = new TalonFX(driveID);
    angleMotor = new TalonFX(angleID);
    nacCoder = new CANcoder(nacID);

    // 0 is default position
    anglePosition.Slot = 0;

    driveOpenLoop = new DutyCycleOut(0);
    driveVelocity = new VelocityVoltage(0);
    driveVelocity.Slot = 0;

    configDriveMotor(driveInvert);
    configAngleMotor();

    driveMotor.setPosition(0.0);
    lastAngle = Rotation2d.fromDegrees(0);
    resetToAbsolute();
  }

  public void configDriveMotor(InvertedValue drive) {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveMotor.getConfigurator().apply(new TalonFXConfiguration());

    driveConfig.MotorOutput.Inverted = drive;
    driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    driveConfig.Feedback.SensorToMechanismRatio = Constants.driveRatio; ///((Math.PI)*Units.inchesToMeters(4))
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = 60;

    driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    driveConfig.Slot0.kP = Constants.driveKp;
    driveConfig.Slot0.kI = Constants.driveKi;
    driveConfig.Slot0.kD = Constants.driveKd;

    driveMotor.getConfigurator().apply(driveConfig);
    driveMotor.getConfigurator().setPosition(0.0);

  }

  public void configAngleMotor() {
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    angleMotor.getConfigurator().apply(new TalonFXConfiguration());

    angleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    angleConfig.Feedback.SensorToMechanismRatio = Constants.angleRatio;
    angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleConfig.ClosedLoopGeneral.ContinuousWrap = true;

    angleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    angleConfig.CurrentLimits.StatorCurrentLimit = 60;

    angleConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    angleConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    angleConfig.Slot0.kP = Constants.angleP;
    angleConfig.Slot0.kI = Constants.angleI;
    angleConfig.Slot0.kD = Constants.angleD;

    angleMotor.getConfigurator().apply(angleConfig);
  }

  public void resetToAbsolute() {
    double absoluteValue = getNac() - offset.getRotations();
    angleMotor.setPosition(absoluteValue);
  }

  public double getNac() {
    return nacCoder.getPosition().getValueAsDouble();
  }

  public double getDrivePosition() {
    return driveMotor.getPosition().getValueAsDouble();
  }

  public double getDriveVel() {
    return driveMotor.getVelocity().getValueAsDouble();
  }

  public double getAngle() {
    return angleMotor.getPosition().getValueAsDouble();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosConversion(), Rotation2d.fromRotations(getAngle()));
  }

  public void resetLastAngle() {
    lastAngle = Rotation2d.fromDegrees(0);
  }

  public void runVelocity(double vel) {
    driveVelocity.Velocity = vel / Constants.wheelCircum; // meters per second to rotations per second


    driveVelocity.Slot = 0;

      // sets the feedforward to simple feedforward calculation with the requested
      // speed
    driveVelocity.FeedForward = driveKfCalc.calculate(vel);

      // Calculate using feedforward
    driveMotor.setControl(driveVelocity);
  }

  public void goToAngle(double deg) {
    anglePosition.withSlot(0);
    // anglePosition.withFeedForward(.2);
    // angleMotor.setControl(anglePosition.withPosition(deg.getRotations()));
    // anglePosition.Position = deg.getRotations();
    // angleMotor.setControl(anglePosition);
    angleMotor.set(-deg);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean openLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

    setSpeed(desiredState, openLoop);
    setAngle(desiredState);
  }

  // get state
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), Rotation2d.fromRotations(getAngle()));
  }

  public void setSpeed(SwerveModuleState desiredState, boolean openLoop) {
    if (openLoop) {
      driveOpenLoop.Output = desiredState.speedMetersPerSecond / Constants.maxSpeed;
      driveMotor.setControl(driveOpenLoop); // change this to a set speed if its not smooth
    }
    // add closed loop for driveController
    else {
      // converts velocity to Rotation per second with wheel curcumfirence
      driveVelocity.Velocity = desiredState.speedMetersPerSecond / Constants.wheelCircum;

      driveVelocity.Slot = 0;

      // sets the feedforward to simple feedforward calculation with the requested
      // speed
      driveVelocity.FeedForward = driveKfCalc.calculate(desiredState.speedMetersPerSecond);

      // Calculate using feedforward
      driveMotor.setControl(driveVelocity);
    }
  }

  public void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle;
    double absSpd = Math.abs(desiredState.speedMetersPerSecond);
    anglePosition.Slot = 0;

    if (absSpd <= (Constants.maxSpeed * .01)) {
      angle = lastAngle;
    } else {
      angle = desiredState.angle;
    }

    anglePosition.Position = angle.getRotations();
    angleMotor.setControl(anglePosition);
    lastAngle = angle;

  }

  public double getDriveVelocityConversion() {
    double meters = driveMotor.getVelocity().getValueAsDouble() * Constants.wheelCircum;
    // SmartDashboard.putString("Units of velocity", driveMotor.getVelocity().getUnits());
    
    return meters;
  }

  public double getDrivePosConversion() {
    double velocity = driveMotor.getPosition().getValueAsDouble() * Constants.wheelCircum;
    // SmartDashboard.putString("Units of velocity", driveMotor.getVelocity().getUnits());
    
    return velocity;
  }

  // public double getAnglePosConversion() {
  //   double angleDeg = angleMotor.getPosition().getValueAsDouble();

    

  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Module Angle " + modNum, getAngle());
    // SmartDashboard.putNumber("Swerve Velocity " + modNum, getDriveVelocityConversion());
    // SmartDashboard.putNumber("Drive Distance " + modNum, getDrivePosConversion());
    // SmartDashboard.putNumber("Drive Velocity Wanted" + modNum, .5); 
    // SmartDashboard.putNumber("Swerve Motor Voltage " + modNum, driveMotor.getMotorVoltage().getValueAsDouble());
    // SmartDashboard.putNumber("Swerve Motor Supply " + modNum, driveMotor.getSupplyVoltage().getValueAsDouble());
    // SmartDashboard.putNumber("Nac Angle " + modNum, getNac());
    // SmartDashboard.putNumber("Rotation 90", Rotation2d.fromDegrees(90).getRotations());
  }
}
