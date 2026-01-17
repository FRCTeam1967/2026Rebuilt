package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopingArm;

public class LowerArmUntilBrake extends Command {
  private TelescopingArm telescopingArm;
  
  public LowerArmUntilBrake(TelescopingArm telescopingArm) {
    this.telescopingArm = telescopingArm;
    addRequirements(telescopingArm);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    if (telescopingArm.isEnabled() && !telescopingArm.getSensorValue()) {
      telescopingArm.runMotor(() -> Constants.TelescopingArm.LOWER_SPEED);
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    telescopingArm.stop();
  }
  
  @Override
  public boolean isFinished() {
    return telescopingArm.getSensorValue();
  }
}