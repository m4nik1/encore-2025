// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainKrakens extends SubsystemBase {
  /** Creates a new TestDrive. */

  ElmCityKrakenModule[] elmCityModules;

  Pigeon2 gyro;

  Pose2d robotPose;
  // SwerveDrivePoseEstimator odom;
  SwerveDriveOdometry odom;

  public DriveTrainKrakens() {
    elmCityModules = new ElmCityKrakenModule[] {
      new ElmCityKrakenModule(0, 12, 13, 24, Constants.angleOffsetMod0, InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityKrakenModule(1, 15, 14, 25, Constants.angleOffsetMod1, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityKrakenModule(2, 1, 2, 26, Constants.angleOffsetMod2, InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityKrakenModule(3, 4, 3, 27, Constants.angleOffsetMod3, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive)
    };

    gyro = new Pigeon2(Constants.pigeonID);
    resetGyro();

    odom = new SwerveDriveOdometry(Constants.swerveKinematics, getYaw(), getPositions());

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        odom.resetPosition(new Rotation2d(), getPositions(), new Pose2d());
      } catch(Exception e) {}
    }).start();

    robotPose = getPose();
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  public Pose2d getRobotPose() {
    // Pose2d pose = poseEstimator.update(getYaw(), getPositions());
    Pose2d p = odom.update(getYaw(), getPositions());
    return p;
    // return pose;
  }

  public void resetPose(Pose2d pose) {
    // poseEstimator.resetPosition(getYaw(), getPositions(), pose);
    odom.resetPosition(getYaw(), getPositions(), pose);
  }

  public Pose2d getPose() {
    return odom.getPoseMeters();
    // return odom.getEstimatedPosition();
  }

  public void setDriveVelocity() {
    for(ElmCityKrakenModule m : elmCityModules) {
      m.runVelocity(1);
    }
  }


  public void driveRobotRelative(ChassisSpeeds spds) {
    SwerveModuleState states[] = Constants.swerveKinematics.toSwerveModuleStates(spds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxSpeed);

    for(ElmCityKrakenModule m : elmCityModules) {
      m.setDesiredState(states[m.modNum], false);
    }
  }


  public void resetGyro() {
    gyro.getConfigurator().setYaw(0.0);

    gyro.reset();
  }

  public void drive(Translation2d translation, double rotation) {
    SwerveModuleState[] moduleStates;

    ChassisSpeeds spds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw());
    spds = ChassisSpeeds.discretize(spds, .02);
    moduleStates = Constants.swerveKinematics.toSwerveModuleStates(spds);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.maxSpeed);

    for(ElmCityKrakenModule m : elmCityModules) {
      m.setDesiredState(moduleStates[m.modNum], true);
    }
  }

  public void setAngle(double deg) {
    // for(ElmCityKrakenModule k : elmCityModules) {
      // elmCityModules[k.modNum].goToAngle(Rotation2d.fromDegrees(deg));
    // }
    elmCityModules[2].goToAngle(0.1);
  }

  public ChassisSpeeds getRobotSpds() {
    return Constants.swerveKinematics.toChassisSpeeds(
      elmCityModules[0].getState(),
      elmCityModules[1].getState(),
      elmCityModules[2].getState(),
      elmCityModules[3].getState()
    );
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for(ElmCityKrakenModule k : elmCityModules) {
      positions[k.modNum] = elmCityModules[k.modNum].getPosition();
    }

    return positions;
  }

  public void resetSwerveAngle() {
    for(ElmCityKrakenModule k : elmCityModules) {
      k.resetLastAngle();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    robotPose = getRobotPose();
  }
}
 