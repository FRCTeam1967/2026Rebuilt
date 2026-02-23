package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.Constants;

public class RunHood extends Command {

  private final Hood hood;
  private final double targetPosRevs;

  public RunHood(Hood hood, double targetPosRevs) {
    this.hood = hood;
    this.targetPosRevs = targetPosRevs;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.moveTo(targetPosRevs); //changed name to reflect parameter type and not cause confusion
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hood.isReached();
  }
}
