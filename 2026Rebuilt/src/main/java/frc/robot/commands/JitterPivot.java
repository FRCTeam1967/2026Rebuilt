// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JitterPivot extends Command {
  private Pivot pivot;
  private int count = 0;
  private Timer timer = new Timer();
 
 
  /** Creates a new JitterPivot. */
  public JitterPivot(Pivot pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.resetRelEncoder();
    timer.start();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() % 1 == 0) {
      if (Math.abs(pivot.getPosition()-Constants.Pivot.UPPER_ARM_POS)<Constants.Pivot.JITTER_DEADBAND){
        pivot.moveTo(Constants.Pivot.LOWER_ARM_POS);
      }
      else if (Math.abs(pivot.getPosition()-Constants.Pivot.LOWER_ARM_POS)<Constants.Pivot.JITTER_DEADBAND){
        pivot.moveTo(Constants.Pivot.UPPER_ARM_POS);
      }
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.stop();
    timer.stop();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
