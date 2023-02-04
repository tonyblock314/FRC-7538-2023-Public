// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Mode;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class setMotorMode extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveTrain m_subsystem;
  private Mode m_mode;

  public setMotorMode(DriveTrain subsystem, Mode mode) {
    m_subsystem = subsystem;
    m_mode = mode;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_subsystem.setMode(m_mode);
  }
  
  @Override
  public boolean isFinished() {
    return true;
  }
}