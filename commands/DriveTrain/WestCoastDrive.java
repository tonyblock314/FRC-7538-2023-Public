// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import frc.robot.subsystems.DriveTrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WestCoastDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveTrain m_subsystem;
  private DoubleSupplier leftStickY, rightStickX;

  public WestCoastDrive(DriveTrain subsystem, DoubleSupplier leftStickY, DoubleSupplier rightStickX, BooleanSupplier xPressed) {
    m_subsystem = subsystem;
    this.leftStickY = leftStickY;
    this.rightStickX = rightStickX;
    addRequirements(subsystem);
  }
  
  @Override
  public void execute() {
    double leftStickYAsDouble = leftStickY.getAsDouble();
    double rightStickXAsDouble = rightStickX.getAsDouble();
    
    /* Uncommented when receive limelight
    double Kp = -0.1;
    double min_command = 0.05;
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    if (xPressed.getAsBoolean()) {
      double heading_error = -tx;
      if (tx > 1.0) {
        rightStickXAsDouble += Kp * heading_error - min_command;
      } else if (tx < 1.0) {
        rightStickXAsDouble += Kp * heading_error + min_command;
      }
      MathUtil.clamp(rightStickXAsDouble, -1, 1);
    }*/

    m_subsystem.arcadeDrive(leftStickYAsDouble, rightStickXAsDouble);
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      m_subsystem.setMotors(0,0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
