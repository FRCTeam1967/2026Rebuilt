// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

import org.wpilib.driverstation.Gamepad;
import org.wpilib.framework.TimedRobot;
import org.wpilib.math.filter.SlewRateLimiter;

import org.wpilib.math.util.*;
import org.wpilib.units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import org.wpilib.framework.OpModeRobot;


public class Robot extends OpModeRobot {
  private final Gamepad controller = new Gamepad(0);
  //private final Drivetrain drive = new Drivetrain();

  private final TalonFX motor = new TalonFX(1, CANBus.systemcore(0)); //TODO: change ID & port

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter velocityLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);


 public Robot() {}

  /** This function is called exactly once when the DS first connects. */
  @Override
  public void driverStationConnected() {}

  /**
   * This function is called periodically anytime when no opmode is selected, including when the
   * Driver Station is disconnected.
   */
  @Override
  public void nonePeriodic() {}
}
