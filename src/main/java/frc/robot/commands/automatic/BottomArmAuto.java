// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatic;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomArm;

import static frc.robot.Constants.*;

public class BottomArmAuto extends CommandBase {
  private BottomArm bottomArm;
  private Supplier<Boolean> upPos, downPos;
  private boolean feedCone, setpointReached;
  private int defaultSetpoint, goalSetpoint, currentSetpoint;
  private double[] setpointsInner = {RETRACTED_INNER, FEED_INNER, PIVOT_INNER, EXTENDED_INNER};
  private double[] setpointsOuter = {RETRACTED_OUTER, FEED_OUTER, PIVOT_OUTER, EXTENDED_OUTER};
  private double error, errorIntegral, dt, previousError, errorDerivative, previousTimestamp;
  private PIDController pidControllerOuter = new PIDController(OUTER_SEG_KP, OUTER_SEG_KI, OUTER_SEG_KD);
  private PIDController pidControllerInner = new PIDController(INNER_SEG_KP, INNER_SEG_KI, INNER_SEG_KD);

  /** Creates a new BottomArmAuto. */
  public BottomArmAuto(BottomArm bottomArm, Supplier<Boolean> upPos, Supplier<Boolean> downPos) {
    addRequirements(bottomArm);
    this.bottomArm = bottomArm;
    this.upPos = upPos;
    this.downPos = downPos;
    pidControllerOuter.setTolerance(4);
    pidControllerInner.setTolerance(4);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousError = 0;
    previousTimestamp = Timer.getFPGATimestamp();

    bottomArm.resetEncoders();
    currentSetpoint = 0;
    setpointReached = true;
    feedCone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (upPos.get() && currentSetpoint < setpointsInner.length-1) {
      currentSetpoint++;
    }
    else if (downPos.get() && currentSetpoint > 0) {
      currentSetpoint--;
    }

    SmartDashboard.putNumber("Current Setpoint", currentSetpoint);

    //setpointReached = setpointReached(); // determined from encoder readings
    // when currentSetpoint should change
    
    //double innerOutput = calculateMotorOutput(bottomArm.getInnerSegEncoderPos(), setpointsInner[currentSetpoint], INNER_SEG_KP, INNER_SEG_KI, INNER_SEG_KD);
    //double outerOutput = calculateMotorOutput(bottomArm.getOuterSegEncoderPos(), setpointsOuter[currentSetpoint], OUTER_SEG_KP, OUTER_SEG_KI, OUTER_SEG_KD);

    double innerOutput = 0;
    double outerOutput = 0;
    if (Math.abs(bottomArm.getInnerSegEncoderPos() - setpointsInner[currentSetpoint]) > 2.5) {
      innerOutput = pidControllerInner.calculate(bottomArm.getInnerSegEncoderPos(), setpointsInner[currentSetpoint]);
      innerOutput = MathUtil.clamp(innerOutput, -.5, .5);
  
    if (Math.abs(bottomArm.getOuterSegEncoderPos() - setpointsOuter[currentSetpoint]) > 2.5) {
      outerOutput = pidControllerOuter.calculate(bottomArm.getOuterSegEncoderPos(), setpointsOuter[currentSetpoint]);
      outerOutput = MathUtil.clamp(outerOutput, -.5, .5);
    }
    bottomArm.move(-innerOutput, -outerOutput);

    bottomArm.feedOutput(-innerOutput, -outerOutput, setpointsInner[currentSetpoint] - bottomArm.getInnerSegEncoderPos(), setpointsOuter[currentSetpoint] - bottomArm.getOuterSegEncoderPos());
    bottomArm.feedShuffleboardValues(feedCone, currentSetpoint, goalSetpoint, defaultSetpoint, setpointsInner[currentSetpoint], setpointsOuter[currentSetpoint], setpointReached);
  }
}

  /*
  private void coneSwitch() {
    feedCone = !feedCone;
    if (feedCone) {
      defaultSetpoint = 0;
    }
    else {
      defaultSetpoint = 1;
    }
  }
  */

  /* 
  private boolean setpointReached() {
    return (Math.abs(bottomArm.getInnerSegEncoderPos() - setpointsInner[currentSetpoint]) <= 5) && (Math.abs(bottomArm.getOuterSegEncoderPos() - setpointsOuter[currentSetpoint]) <= 5);
  }
  */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double calculateMotorOutput(double position, double setpoint, double kP, double kI, double kD) {
    error = setpoint - position;
    dt = Timer.getFPGATimestamp() - previousTimestamp;
    if (Math.abs(error) < 100) { errorIntegral = error * dt; } // integral term only calculated within a radius to minimize oscillation
    errorDerivative = (error - previousError) / dt; // de/dt

    previousError = error; // update value for next iteration
    previousTimestamp = Timer.getFPGATimestamp();

    return (kP * error) + (kI * errorIntegral) + (kD * errorDerivative);
  }
}