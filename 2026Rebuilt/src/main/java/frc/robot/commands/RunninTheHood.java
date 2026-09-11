package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TheHood;

public class RunninTheHood extends Command {

  private final TheHood hood;
  private final double targetPosRevs;

  /** Creates a new RunninTheHood.
   * @param hood - Hood subsystem
   * @param targetPosRevs - Requested revolutions for moving the hood
   */
  public RunninTheHood(TheHood hood, double targetPosRevs) {
    this.hood = hood;
    this.targetPosRevs = targetPosRevs;
    addRequirements(hood);
  }

  /**Called when the command is initially scheduled.*/
  @Override
  public void initialize() {}

  /**Called every time to update the hood's position to its target angle in revolutions*/
  @Override
  public void execute() {
    hood.moveTo(targetPosRevs);
  }

  /**Called once the command ends or is interrupted.*/
  @Override
  public void end(boolean interrupted) {}

  /**
   * @return true when the command is finished
  */
  @Override
  public boolean isFinished() {
    return false;
  }
}
