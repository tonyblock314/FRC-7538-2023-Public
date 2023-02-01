// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wait extends CommandBase {

  // Instantiates required subsystem and also timer variables that keep track of how long the subsystem has waited
  SubsystemBase m_subsystem;
  long m_startTime;
  long m_currentTime; 
  long m_waitTime;
  boolean isFinished = false;

  /**
   * Creates a new WaitCommand.
   */
  public Wait(SubsystemBase subsystem, int milliseconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_waitTime = (long) milliseconds;
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Sets the start time to the system time at the time the command was scheduled
    // m_currentTime = System.currentTimeMillis();
    m_startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Updates current time every execution of the command
    m_currentTime = System.currentTimeMillis();
    long m_differential = m_currentTime - m_startTime;
    SmartDashboard.putNumber("Time Differential (Subsystem = " + m_subsystem.getClass().getName() + "): ", (double) m_differential);
    isFinished = m_differential >= m_waitTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Command only finishes execution when the change in system time (aka time waited) exceeds the required wait time
    return isFinished;
  }
}