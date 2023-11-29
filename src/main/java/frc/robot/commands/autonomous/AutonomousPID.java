// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.MecanumDrivetrain;
import frc.robot.subsystems.TopArm;

import static frc.robot.Constants.*;

public class AutonomousPID extends CommandBase {
  private double error, dt, errorDerivative, previousTimestamp, previousError, errorIntegral;
  //private double lastWorldLinearAccelX, lastWorldLinearAccelY;
  private double armMotorOutput, forwardOutput, rotation;
  private MecanumDrivetrain drivetrain;
  private TopArm topArm;
  //private BottomArm bottomArm;
  private int phase;
  private AHRS ahrs;
  private Timer timer = new Timer();

  /** Creates a new AutonomousPID. */
  public AutonomousPID(MecanumDrivetrain drivetrain, TopArm topArm, BottomArm bottomArm, AHRS gyroscope) {
    addRequirements(drivetrain, topArm, bottomArm);
    this.drivetrain = drivetrain;
    this.topArm = topArm;
    //this.bottomArm = bottomArm;
    this.ahrs = gyroscope;
  }

  @Override
  public void initialize() {
    phase = 1;
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidAutoSequence();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  private double pidCalculation(double current, double setpoint, double kP, double kI, double kD) {
    error = setpoint - current;
    dt = Timer.getFPGATimestamp() - previousTimestamp;
    if (Math.abs(error) < 100) { errorIntegral = error * dt; } // integral term only calculated within a radius to minimize oscillation
    errorDerivative = (error - previousError) / dt; // de/dt

    previousError = error; // update value for next iteration
    previousTimestamp = Timer.getFPGATimestamp();

    return (kP * error) + (kI * errorIntegral) + (kD * errorDerivative);
  }

  /* 
  private boolean collisionDetected() {
    boolean collisionDetected = false;
    // x-component
    double currentWorldLinearAccelX = ahrs.getWorldLinearAccelX();
    double jerkX = currentWorldLinearAccelX - lastWorldLinearAccelX;
    lastWorldLinearAccelX = currentWorldLinearAccelX;
    // y-component
    double currentWorldLinearAccelY = ahrs.getWorldLinearAccelY();
    double jerkY = currentWorldLinearAccelY - lastWorldLinearAccelY;
    lastWorldLinearAccelY = currentWorldLinearAccelY;
    
    if (Math.abs(jerkX) > COLLISION_THRESHOLD || Math.abs(jerkY) > COLLISION_THRESHOLD) {
      collisionDetected = true;
    }

    return collisionDetected;
  }
  */

  private void pidAutoSequence() {
    switch (phase) {
      case 1: // extend arm
        armMotorOutput = pidCalculation(topArm.getTopEncoderPosition(), ARM_HIGH_SETPOINT, ARM_KP, ARM_KI, ARM_KD);
        topArm.move(armMotorOutput);
        if (Math.abs(armMotorOutput) > .025) {
          phase++;
        }
        break;
      case 2: // move to scoring area
        forwardOutput = pidCalculation(drivetrain.getFrontLeftEncoderPosition(), AUTON_DISTANCE_SETPOINT, AUTON_KP, AUTON_KI, AUTON_KD);
        rotation = pidCalculation(ahrs.getRotation2d().getDegrees(), 0, ROTATE_KP, ROTATE_KI, ROTATE_KD);
        drivetrain.driveCartesian(forwardOutput, 0, rotation, ahrs.getRotation2d());
        if (Math.abs(forwardOutput) < 0.025) {
          phase++;
        }
        break;
      case 3: // release cone
        timer.start();
        if (timer.get() <= 1) {
          topArm.releaseObject(TOP_INTAKE_SPEED);
        }
        else {
          timer.stop();
          phase++;
        }
        break;
      case 4:
        forwardOutput = pidCalculation(drivetrain.getFrontLeftEncoderPosition(), 0, AUTON_KP, AUTON_KI, AUTON_KD);
        rotation = pidCalculation(ahrs.getRotation2d().getDegrees(), 0, ROTATE_KP, ROTATE_KI, ROTATE_KD);
        drivetrain.driveCartesian(forwardOutput, 0, rotation, ahrs.getRotation2d());
        if (Math.abs(forwardOutput) < 0.025) {
          phase++;
        }
        break;
      case 5:
        armMotorOutput = pidCalculation(topArm.getTopEncoderPosition(), ARM_IN_SETPOINT, ARM_KP, ARM_KI, ARM_KD);
        topArm.move(armMotorOutput);
        if (Math.abs(armMotorOutput) > 0.025) {
          phase++;
        }
        break;
      default:
        topArm.move(0);
        topArm.releaseObject(0);
        drivetrain.driveCartesian(0, 0, 0, ahrs.getRotation2d());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
