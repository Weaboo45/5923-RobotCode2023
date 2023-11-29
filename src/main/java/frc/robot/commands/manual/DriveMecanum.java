/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manual;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MecanumDrivetrain;

import static frc.robot.Constants.*;

public class DriveMecanum extends CommandBase {
  /*
   * Creates a new DriveMecanum.
   */

  private MecanumDrivetrain drivetrain;
  private Supplier<Double>  y, x, z, rightY;
  private Supplier<Boolean> motorToggleButton, applyBoostButton;
  private double error, dt, previousTimestamp, previousError, errorIntegral, errorDerivative;
  private double multiplier;
  private Supplier<AHRS> ahrs;
  private boolean boostSwitch, angleLockSwitch, joystickTriggered0, joystickTriggered180, joystickTriggered, joystickTriggeredPrevious;

  public DriveMecanum(MecanumDrivetrain drivetrain, Supplier<Double> forward, Supplier<Double> strafe, Supplier<Double> zRotation, Supplier<AHRS> ahrs, Supplier<Boolean> motorToggleButton, Supplier<Boolean> applyBoostButton, Supplier<Double> rightY) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.y = forward;
    this.x = strafe;
    this.z = zRotation;
    this.ahrs = ahrs;
    this.motorToggleButton = motorToggleButton; // toggle
    this.applyBoostButton = applyBoostButton; // toggle
    this.rightY = rightY;
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boostSwitch = false;
    drivetrain.configureMotorPower();
    angleLockSwitch = false;
    joystickTriggered0 = false;
    joystickTriggeredPrevious = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ySpeed = -y.get();
    double xSpeed = x.get();
    double zRotation = z.get();
    Rotation2d gyroAngle = ahrs.get().getRotation2d();

    if (applyBoostButton.get()) {
      boostSwitch = !boostSwitch;
    }

    if (boostSwitch) {
      multiplier = BOOST_MULTIPLIER;
    }
    else {
      multiplier = 1;
    }

    if (motorToggleButton.get()) {
      drivetrain.toggleMotorMode(true);
    }

    /*
    if (rightY.get() <= -0.95) { joystickTriggered0 = true; }
    else { joystickTriggered0 = false; }

    if (rightY.get() >= 0.95) { joystickTriggered180 = true; }
    else { joystickTriggered180 = false; }

    if (joystickTriggered0 || joystickTriggered180) { joystickTriggered = true; }
    else { joystickTriggered = false; }

    if (joystickTriggered && joystickTriggered != joystickTriggeredPrevious) {
      angleLockSwitch = !angleLockSwitch;
    }
    joystickTriggeredPrevious = joystickTriggered;

    if (!angleLockSwitch) {
      drivetrain.driveCartesian(ySpeed * multiplier, xSpeed * multiplier, zRotation, gyroAngle.times(-1));
    }
    else {
      double rotationOutput = calculateRotationOutput(ahrs.get().getYaw(), angleLockSetpoint(), ROTATE_KP, ROTATE_KI, ROTATE_KD);
      drivetrain.driveCartesian(ySpeed * multiplier, xSpeed * multiplier, rotationOutput, gyroAngle.times(-1));
    }
    */

    drivetrain.driveCartesian(ySpeed * multiplier, xSpeed * multiplier, zRotation, gyroAngle.times(-1));
    
    //drivetrain.driveCartesian(xSpeed, ySpeed, zRotation); // bot-oriented drive
  }

  private double angleLockSetpoint() {  
    if (joystickTriggered0) {
      return 0.0;
    }
    if (joystickTriggered180 && ahrs.get().getYaw() > 0) {
      return 180.0;
    }
    if (joystickTriggered180 && ahrs.get().getYaw() < 180) {
      return -180.0;
    }
    else {
      return 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {;
    drivetrain.driveCartesian(0.0, 0.0, 0.0, ahrs.get().getRotation2d());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double calculateRotationOutput(double angle, double setpoint, double kP, double kI, double kD) {
    error = setpoint - angle;
    dt = Timer.getFPGATimestamp() - previousTimestamp;
    if (Math.abs(error) < 100) { errorIntegral = error * dt; } // integral term only calculated within a radius to minimize oscillation
    errorDerivative = (error - previousError) / dt; // de/dt

    previousError = error; // update value for next iteration
    previousTimestamp = Timer.getFPGATimestamp();

    return (kP * error) + (kI * errorIntegral) + (kD * errorDerivative);
  }
}