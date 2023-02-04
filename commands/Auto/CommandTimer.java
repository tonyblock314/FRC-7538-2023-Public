// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

public class CommandTimer {

  // Instantiates required subsystem and also timer variables that keep track of how long the subsystem has waited
  long m_startTime;
  long m_waitTime;
  boolean isFinished = false;

  /**
   * Creates a new WaitCommand.
   */

  public CommandTimer() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_startTime = System.currentTimeMillis();
    m_waitTime = 1000000;
  }

  public CommandTimer(int milliseconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_startTime = System.currentTimeMillis();
    m_waitTime = (long) milliseconds;
  }

  public void resetTimer(int milliseconds) {
    m_startTime = System.currentTimeMillis();
    m_waitTime = (long) milliseconds;
  }

  public boolean isFinished() {
    // Updates current time every execution of the command
    long m_differential = System.currentTimeMillis() - m_startTime; // Amount of time that has passed
    return m_differential >= m_waitTime; // Allow command to finish if the command has waited the wait time out
  }
}