// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/** An example command that uses an example subsystem. */
public class AutoDrive extends PIDCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoDrive(DriveTrain subsystem, double distance) {
    super(
      new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD),
      // Close loop on heading
      subsystem::getEncoderAverage,
      // Set reference to target
      -distance,
      // Pipe output to turn robot
      output -> subsystem.setMotors(
        Math.min(output, Constants.SCALING_FACTOR * Constants.MAX_DRIVE_SPEED), 
        Math.min(output, Constants.SCALING_FACTOR * Constants.MAX_DRIVE_SPEED)),
      // Require the drive
      subsystem);
    getController()
      .setTolerance(Constants.kDrivePTol, Constants.kDriveDTol);

    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetEncoders();
    // m_drivetrain.resetHeading();
    super.initialize();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
