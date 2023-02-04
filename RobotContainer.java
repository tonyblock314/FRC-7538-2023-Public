// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// All Drive Train commands and subsystems
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.DriveTrain.WestCoastDrive;
import frc.robot.commands.DriveTrain.ZOOM;
import frc.robot.commands.Elevator.UpAndDown;

public class RobotContainer {
  // Create subsystems
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Elevator m_elevator = new Elevator();
  // Instantiate driver controller
  public static XboxController driver = new XboxController(Constants.DRIVER_XBOX_PORT);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    m_driveTrain.setDefaultCommand(new WestCoastDrive(
      m_driveTrain,
      driver::getLeftY,
      driver::getRightX,
      driver::getXButtonPressed));
    m_elevator.setDefaultCommand(new UpAndDown(
      m_elevator,
      driver::getRightBumperPressed,
      driver::getLeftBumperPressed));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Create buttons to link to commands
    JoystickButton xboxControllerLeftStickButton = new JoystickButton(driver, Constants.XBOX_LEFT_STICK_BUTTON);
    JoystickButton xboxControllerRightBumper = new JoystickButton(driver, Constants.XBOX_RIGHT_BUMPER);
    JoystickButton xboxControllerLeftBumper = new JoystickButton(driver, Constants.XBOX_LEFT_BUMPER);
    // Link triggers to commands

    // Link buttons to Commands
    xboxControllerLeftStickButton.whileHeld(new ZOOM(m_driveTrain));
    xboxControllerRightBumper.whileHeld(new UpAndDown(m_elevator));
  }

  public Command getAutonomousCommand() {
    return null;
    /*
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          Constants.kVoltsPos,
          Constants.kVoltsVel,
          Constants.kVoltsAcc),
          DriveTrain.kDriveKinematics,
          10);

    TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.MAX_SPEED_IN_METERS_PER_SECOND,
            Constants.MAX_ACCEL_IN_METERS_PER_SECOND)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveTrain.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);
          
    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);
    
    RamseteCommand ramseteCommand =
    new RamseteCommand(
        exampleTrajectory,
        m_driveTrain::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
          Constants.kVoltsPos,
          Constants.kVoltsVel,
          Constants.kVoltsAcc),
          DriveTrain.kDriveKinematics,
          m_driveTrain::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel, 0, 0),
          new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_driveTrain::tankDriveVolts,
        m_driveTrain);*/

    // A RamseteCommand will run in autonomous
    //return ramseteCommand.andThen(() -> m_driveTrain.tankDriveVolts(0, 0))
  }
}
