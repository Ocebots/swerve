// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.CANMappings;
import frc.constants.DriveConstants;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule frontLeft =
      new MAXSwerveModule(
          CANMappings.FRONT_LEFT_DRIVING,
          CANMappings.FRONT_LEFT_TURNING,
          DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final MAXSwerveModule frontRight =
      new MAXSwerveModule(
          CANMappings.FRONT_RIGHT_DRIVING,
          CANMappings.FRONT_RIGHT_TURNING,
          DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private final MAXSwerveModule rearLeft =
      new MAXSwerveModule(
          CANMappings.REAR_LEFT_DRIVING,
          CANMappings.REAR_LEFT_TURNING,
          DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final MAXSwerveModule rearRight =
      new MAXSwerveModule(
          CANMappings.REAR_RIGHT_DRIVING,
          CANMappings.REAR_RIGHT_TURNING,
          DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

  // The gyro sensor
  private final AHRS gyro = new AHRS();

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.ROTATIONAL_SLEW_RATE);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          DriveConstants.DRIVE_KINEMATICS,
          Rotation2d.fromDegrees(this.gyro.getAngle()),
          new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.rearLeft.getPosition(),
            this.rearRight.getPosition()
          });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    this.addChild("Front Left", frontLeft);
    this.addChild("Front Left", frontLeft);
    this.addChild("Rear Right", rearRight);
    this.addChild("Rear Right", rearRight);

    this.addChild("Gyro", gyro);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    this.odometry.update(
        Rotation2d.fromDegrees(this.gyro.getAngle()),
        new SwerveModulePosition[] {
          this.frontLeft.getPosition(),
          this.frontRight.getPosition(),
          this.rearLeft.getPosition(),
          this.rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return this.odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    this.odometry.resetPosition(
        Rotation2d.fromDegrees(this.gyro.getAngle()),
        new SwerveModulePosition[] {
          this.frontLeft.getPosition(),
          this.frontRight.getPosition(),
          this.rearLeft.getPosition(),
          this.rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar(theta and magnitude) for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      // if driving set the rotation slew rate to a constant divided by current translation speed so
      // the faster you are, the slower you turn
      // else practically don't limit rotational speed
      double directionSlewRate;
      if (this.currentTranslationMag != 0.0) {
        directionSlewRate =
            Math.abs(DriveConstants.DIRECTION_SLEW_RATE / this.currentTranslationMag);
      } else {
        directionSlewRate =
            500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - this.prevTime;
      double angleDif =
          SwerveUtils.angleDifference(inputTranslationDir, this.currentTranslationDir);

      /*
       * Description of the if below
       *
       * if the difference between the target direction and current direction is smallish,
       * step the current direction slowly to match the input based on the slew rate and time.
       * Allow the translation speed to change based on the limiter. This will result in the robot turning while it maintains speed
       *
       * if the difference is large and the robot is moving, slow down
       * if stopped swap the direction and accelerate away.
       * This performs a 180 flip that is then corrected by the first case. The robot stops, then accelerates backward
       *
       * if the difference is in the middle, step the current angle towards the target and slow the robot.
       * This will eventually switch to the first case when the robot accelerates again.
       */

      // if angle is less than 81 degrees, step the current translation direction to the input with
      // a step size the size of the slew rate, this will prevent the dir from changing too fast
      // then limit the translation speed based on the limiter
      if (angleDif < 0.45 * Math.PI) {
        this.currentTranslationDir =
            SwerveUtils.stepTowardsCircular(
                currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        this.currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        // if the angle is greater than 153 degrees and the requested translation speed is non-zero
        // slow the robot speed to zero
        if (this.currentTranslationMag
            > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          this.currentTranslationMag = magLimiter.calculate(0.0);
        } else {
          // if the robot speed is already at zero flip the angle so the robot reverses direction
          // and begin to re-accelerate
          this.currentTranslationDir = SwerveUtils.wrapAngle(currentTranslationDir + Math.PI);
          this.currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      } else {
        // if the angle is between the two checks, slow the robot to zero and begin to turn
        this.currentTranslationDir =
            SwerveUtils.stepTowardsCircular(
                currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        this.currentTranslationMag = magLimiter.calculate(0.0);
      }
      this.prevTime = currentTime;

      xSpeedCommanded = this.currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = this.currentTranslationMag * Math.sin(currentTranslationDir);
      this.currentRotation = rotLimiter.calculate(rot);
    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      this.currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double rotDelivered = this.currentRotation * DriveConstants.MAX_ANGULAR_SPEED;

    var swerveModuleStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(this.gyro.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    this.frontLeft.setDesiredState(swerveModuleStates[0]);
    this.frontRight.setDesiredState(swerveModuleStates[1]);
    this.rearLeft.setDesiredState(swerveModuleStates[2]);
    this.rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    this.frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    this.frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    this.rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    this.rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    this.frontLeft.setDesiredState(desiredStates[0]);
    this.frontRight.setDesiredState(desiredStates[1]);
    this.rearLeft.setDesiredState(desiredStates[2]);
    this.rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    this.frontLeft.resetEncoders();
    this.rearLeft.resetEncoders();
    this.frontRight.resetEncoders();
    this.rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    this.gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(this.gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return this.gyro.getRate() * (DriveConstants.GYRO_IS_REVERSED ? -1.0 : 1.0);
  }
}
