// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Gyro stuff we're not using ;-;
/*import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import com.ctre.phoenix.sensors.WPI_Pigeon2;*/

// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;
import frc.robot.filters.AdjustableSlewRateLimiter;

public class DriveTrain extends SubsystemBase {

  // Instantiates motors that subsystem controls to link code to CAN bus
  WPI_TalonFX leftA = new WPI_TalonFX(Constants.LEFT_1_CAN_ID);
  WPI_TalonFX leftB = new WPI_TalonFX(Constants.LEFT_2_CAN_ID);
  WPI_TalonFX rightA = new WPI_TalonFX(Constants.RIGHT_1_CAN_ID);
  WPI_TalonFX rightB = new WPI_TalonFX(Constants.RIGHT_2_CAN_ID);

  // Instantiates Pigeon 2.0 IMU

  /*WPI_Pigeon2 pigeonGyro = new WPI_Pigeon2(Constants.PIGEON_CAN_ID);
  DifferentialDriveOdometry m_odometry;
  public static DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(Constants.TRACK_WIDTH_METERS);*/

  // Create chooser for accel limiting
  //SendableChooser<Boolean> m_limit_chooser = new SendableChooser<>();
  //boolean m_currentChoice = false;

  // Creates slew rate limiters in case we decide to limit acceleration for motor protection
  AdjustableSlewRateLimiter leftInputLimiter = new AdjustableSlewRateLimiter(Constants.DRIVE_TRAIN_RATE_LIMIT);
  AdjustableSlewRateLimiter rightInputLimiter = new AdjustableSlewRateLimiter(Constants.DRIVE_TRAIN_RATE_LIMIT);

  /*
  *   Code that initializes the drive train
  */

  public DriveTrain() {
    // Inverts right motors so that positive inputs move robot forward
    invertRightMotors(true);
    setMode(Mode.COAST);

    leftB.follow(leftA);
    rightB.follow(rightA);

    //m_limit_chooser.setDefaultOption("No limit", false);
    //m_limit_chooser.addOption("Limited", true);

    // m_odometry = new DifferentialDriveOdometry(getHeading());
  }

  public void invertRightMotors(boolean isInverted) {
    rightA.setInverted(isInverted);
    rightB.setInverted(isInverted);
  }

  public static enum Mode {
    COAST,
    BRAKE
  }

  public void setMode(Mode mode) {
    switch(mode) {
      case COAST:
        leftA.setNeutralMode(NeutralMode.Coast);
        leftB.setNeutralMode(NeutralMode.Coast);
        rightA.setNeutralMode(NeutralMode.Coast);
        rightB.setNeutralMode(NeutralMode.Coast);
        break;
      case BRAKE:
        leftA.setNeutralMode(NeutralMode.Brake);
        leftB.setNeutralMode(NeutralMode.Brake);
        rightA.setNeutralMode(NeutralMode.Brake);
        rightB.setNeutralMode(NeutralMode.Brake);
        break;
    }
  }

  /*
  *   Code that handles the driving
  */

  // Yes, I know I can implement Differential Drive and just use arcadeDrive(frw, rot), I'm just bored plz don't judge
  public void arcadeDrive(double leftStickY, double rightStickX) {
    // Arcade-style driving; left stick forward/backward for driving speed, right stick left/right for turning
    double leftStickY_DB = deadBand(leftStickY, Constants.LY_DEADBAND);
    double rightStickX_DB = deadBand(rightStickX, Constants.RX_DEADBAND) * Constants.TURN_FACTOR;
    // No circular limitations because LY and RX are on different sticks, but scaling factor to prevent values outside of [-1,1]
    double leftInput = Constants.SCALING_FACTOR * (leftStickY_DB - rightStickX_DB);
    double rightInput = Constants.SCALING_FACTOR * (leftStickY_DB + rightStickX_DB);
    // Display numbers to SmartDashboard to help inform drivers, especially during troubleshooting tests
    SmartDashboard.putNumber("Left Motor Speeds: ", leftInput);
    SmartDashboard.putNumber("Right Motor Speeds: ", rightInput);
    // Update acceleration limit to option selected on SmartDashboard
    // leftInputLimiter.setRateLimit(SmartDashboard.getNumber("", defaultValue));
    /*if (m_limit_chooser.getSelected() != m_currentChoice) {
      m_currentChoice = m_limit_chooser.getSelected();
      leftInputLimiter.reset(leftInput);
      rightInputLimiter.reset(rightInput);
    }
    // Sets motor speeds
    if (m_limit_chooser.getSelected()) {
      setMotors(leftInputLimiter.calculate(leftInput), rightInputLimiter.calculate(rightInput));
    } else {
      setMotors(leftInput, rightInput);
    }*/
    setMotors(leftInput, rightInput);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftA.setVoltage(leftVolts);
    rightA.setVoltage(rightVolts);
  }

  public double deadBand(double speed, double deadBand) {
    // Sets speeds to 0 if within deadband
    if(speed == 0 || Math.abs(speed) < deadBand) {
      return 0;
    }
    // Dilate to use whole range of speeds; obfuscated a lil' bit
    return speed * (1 - deadBand / Math.abs(speed)) / (1 - deadBand);
  }

  public void setMotors(double lSpeed, double rSpeed) {
    // Pushes new speeds to drive train using set() method
    leftA.set(lSpeed);
    rightA.set(rSpeed);
  }

  /*
  *   Handles encodes for PID and also misc. utilities
  */

  public void resetEncoders() {
    leftA.setSelectedSensorPosition(0);
    rightA.setSelectedSensorPosition(0);
  }

  public double getEncoderPosition(WPI_TalonFX talon) {
    return talon.getSelectedSensorPosition() * Constants.RAW_SENSOR_UNITS_TO_DRIVE_METERS;
  }

  public double getEncoderAverage() { // good enough for forwards / backwards
    double leftPos = leftA.getSelectedSensorPosition();
    double rightPos = rightA.getSelectedSensorPosition();
    double sensorUnitAverage = (leftPos + rightPos) / 2;
    double ret = sensorUnitAverage * Constants.RAW_SENSOR_UNITS_TO_DRIVE_METERS;
    updateDashboard();
    SmartDashboard.putNumber("AutoDist: ", ret);
    return ret;
    // Note: 2048 units per rotation; use gear ratio to find distance travelled per raw sensor unit
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("leftEncoder (meters): ", leftA.getSelectedSensorPosition() * Constants.RAW_SENSOR_UNITS_TO_DRIVE_METERS);
    SmartDashboard.putNumber("rightEncoder (meters): ", rightA.getSelectedSensorPosition() * Constants.RAW_SENSOR_UNITS_TO_DRIVE_METERS);
    SmartDashboard.putNumber("Talon ID1 Temperature: ", leftA.getTemperature());
    SmartDashboard.putNumber("Talon ID2 Temperature: ", leftB.getTemperature());
    SmartDashboard.putNumber("Talon ID3 Temperature: ", rightA.getTemperature());
    SmartDashboard.putNumber("Talon ID4 Temperature: ", rightB.getTemperature());
  }

  /*
  *   Handles gyro
  */

  /*public void resetHeading() {
    pigeonGyro.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(pigeonGyro.getAngle());
  }

  public double getDegrees() {
    return pigeonGyro.getAngle();
  }

  public double getTurnRate() {
    return -pigeonGyro.getRate();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, pigeonGyro.getRotation2d());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftA.getSelectedSensorVelocity() * Constants.RAW_SENSOR_UNITS_TO_METERS, 
      rightA.getSelectedSensorVelocity() * Constants.RAW_SENSOR_UNITS_TO_METERS);
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*m_odometry.update(
      getHeading(), getEncoderPosition(leftA), getEncoderPosition(rightA));*/
  }
}
