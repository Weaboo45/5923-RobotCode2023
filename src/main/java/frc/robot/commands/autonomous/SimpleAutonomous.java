// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.MecanumDrivetrain;
import frc.robot.subsystems.TopArm;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

public class SimpleAutonomous extends CommandBase {
  private MecanumDrivetrain drivetrain;
  private TopArm topArm;
  //private BottomArm bottomArm;
  private int phase;
  private AHRS ahrs;
  private Timer timer = new Timer();

  /** Creates a new SimpleAutonomous. */
  public SimpleAutonomous(MecanumDrivetrain drivetrain, TopArm topArm, BottomArm bottomArm, AHRS gyroscope) {
    addRequirements(drivetrain, topArm, bottomArm);
    this.drivetrain = drivetrain;
    this.topArm = topArm;
    //this.bottomArm = bottomArm;
    this.ahrs = gyroscope;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    phase = 1;
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timedAutoSequence();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  private void timedAutoSequence() {
    switch (phase) {
      case 1: // extend arm
        if (timer.get() < 3) {
          topArm.move(0.4);
        }
        else {
          topArm.move(0);
          phase++;
        }
        break;
      case 2: // release cube
        if (timer.get() < 4) {
          topArm.releaseObject(TOP_INTAKE_SPEED);
        }
        else {
          topArm.releaseObject(0);
          phase++;
        }
        break;
      case 3: // drive back, retract arm
        if (timer.get() < 6) {
          drivetrain.driveCartesian(.9, 0, 0, ahrs.getRotation2d());
          topArm.move(-0.28);
        }
        else {
          drivetrain.driveCartesian(0, 0, 0, ahrs.getRotation2d());
          phase++;
        }
        break;
      default:
        topArm.move(0);
        topArm.releaseObject(0);
      }
    }
  


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
