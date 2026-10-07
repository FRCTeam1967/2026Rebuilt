// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.command2.Command;
import org.wpilib.hardware.power.PowerDistribution;

import frc.robot.subsystems.Eater;
import frc.robot.BatteryParam.BatteryParamEstimator;
import frc.robot.MotorCurrentEstimators.CIMCurrentEstimator;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunEater extends Command {
  public Eater eater;
  private double speed;
  public double maxCurrent;
  public double estCurrent;
  private static final PowerDistribution pdh = new PowerDistribution(0); //TODO: determine bus id
  public BatteryParamEstimator batteryEstimator = new BatteryParamEstimator(100);
  public CIMCurrentEstimator currentEstimator = new CIMCurrentEstimator(1, 0.1, pdh);;


  /** Creates a new RunIntake. */
  public RunEater(Eater eater, double speed) {
    this.eater = eater;
    this.speed = speed;
    addRequirements(eater);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    batteryEstimator.updateEstimate(pdh.getVoltage(), pdh.getTotalCurrent());//feed the battery estimator the latest voltage and current readings
    maxCurrent = batteryEstimator.getMaxIdraw(7.0);//estimates the max current which may be drawn from the battery
    estCurrent = currentEstimator.getCurrentEstimate(eater.getVelocityRPS() * 2 * Math.PI, speed / Constants.Eater.KRAKEN_MAX_RPS); // estimates current draw based on velocity in radians per second and normalized speed    
    double limitedSpeed = speed;

    if (estCurrent > maxCurrent) {
      limitedSpeed = speed * (maxCurrent / estCurrent); 
  }

  eater.setMotor(limitedSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    eater.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
