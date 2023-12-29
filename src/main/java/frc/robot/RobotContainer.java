// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.constants.AutoConstants;
import frc.constants.ControllerConstants;
import frc.constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();

  // The driver's controller
  CommandXboxController driverController =
      new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    this.robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        Commands.run(
            () ->
                robotDrive.drive(
                    -MathUtil.applyDeadband(
                        driverController.getLeftY(), ControllerConstants.DRIVE_DEADBAND),
                    -MathUtil.applyDeadband(
                        driverController.getLeftX(), ControllerConstants.DRIVE_DEADBAND),
                    -MathUtil.applyDeadband(
                        driverController.getRightX(), ControllerConstants.DRIVE_DEADBAND),
                    true,
                    true),
            robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    this.driverController.a().onTrue(Commands.runOnce(() -> this.robotDrive.zeroHeading()));
    this.driverController
        .rightBumper()
        .whileTrue(Commands.run(() -> this.robotDrive.setX(), this.robotDrive));
  }

  public void log() {
    this.robotDrive.logData();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.DRIVE_KINEMATICS);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.P_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            this.robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.DRIVE_KINEMATICS,

            // Position controllers
            new PIDController(AutoConstants.PX_CONTROLLER, 0, 0),
            new PIDController(AutoConstants.PY_CONTROLLER, 0, 0),
            thetaController,
            this.robotDrive::setModuleStates,
            this.robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    this.robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> this.robotDrive.drive(0, 0, 0, false, false));
  }
}
