// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.filters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

/**
 * Programmer note: 98% of this file was copied from the SlewRateLimiter class provided by FRC. All credit to FRC. FRC is the OG :D
 */
public class AdjustableSlewRateLimiter {
  private double m_rateLimit;
  private double m_prevVal;
  private double m_prevTime;

  public AdjustableSlewRateLimiter(double rateLimit, double initialValue) {
    m_rateLimit = rateLimit;
    m_prevVal = initialValue;
    m_prevTime = WPIUtilJNI.now() * 1e-6;
  }

  public AdjustableSlewRateLimiter(double rateLimit) {
    this(rateLimit, 0);
  }

  public double calculate(double input) {
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    m_prevVal +=
        MathUtil.clamp(input - m_prevVal, -m_rateLimit * elapsedTime, m_rateLimit * elapsedTime);
    m_prevTime = currentTime;
    return m_prevVal;
  }

  public void reset(double value) {
    m_prevVal = value;
    m_prevTime = WPIUtilJNI.now() * 1e-6;
  }

  public void setRateLimit(double value) {
    m_rateLimit = value;
  }
}