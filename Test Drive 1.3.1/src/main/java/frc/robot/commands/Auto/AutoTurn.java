// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.commands.Auto;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
*/
/** An example command that uses an example subsystem. */
/*
public class AutoTurn extends PIDCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_subsystem;

  *//**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   *//*
  public AutoTurn(DriveTrain subsystem, double degrees, double currentDegrees) {
    super(
      new PIDController(Constants.kTurnP, Constants.kTurnI, Constants.kTurnD),
      // Close loop on heading
      () -> subsystem.getDegrees() - currentDegrees,
      // Set reference to target
      degrees,
      // Pipe output to turn robot
      output -> subsystem.setMotors(
        Math.min(output, Constants.TURN_FACTOR * Constants.SCALING_FACTOR * Constants.MAX_DRIVE_SPEED), // Turn right increases deg -> left move forward
        -Math.min(output, Constants.TURN_FACTOR * Constants.SCALING_FACTOR * Constants.MAX_DRIVE_SPEED)), // Caps respective to max turning speed in teleop
      // Require the drive
      subsystem);
    getController()
      .setTolerance(Constants.kTurnPTol, Constants.kTurnDTol);

    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetHeading();
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
}*/
