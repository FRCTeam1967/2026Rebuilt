// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FlywheelShooter;

public class Simulation extends SubsystemBase {
  // Size in "meters" for the mechanism canvas (just a drawing coordinate system)
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(3, 3);

  // Keep references so we can animate angles later
  private final LoggedMechanismLigament2d hoodLigament;
  private final LoggedMechanismLigament2d flywheelLigament;

  /** Creates a new Simulation. */
  public Simulation() {
    // Root point where the shooter is mounted
    LoggedMechanismRoot2d root = mechanism.getRoot("ShooterRoot", 1.5, 0.8);

    // Hood/arm that can rotate (THIS is the important part: LoggedMechanismLigament2d)
    hoodLigament =
        root.append(
            new LoggedMechanismLigament2d(
                "Hood",
                1.0,   // length
                35.0   // starting angle (degrees)
            ));

    // A second ligament attached to the hood (just to prove 3D output works)
    flywheelLigament =
        hoodLigament.append(
            new LoggedMechanismLigament2d(
                "Flywheel",
                0.6,   // length
                0.0    // relative angle
            ));
  }

  @Override
  public void periodic() {
    // ✅ This is what AdvantageScope needs. It will auto-generate Mechanism3d internally.
    Logger.recordOutput("Shooter/Mechanism2d", mechanism);

    // --- OPTIONAL: simple animation so you can SEE it moving immediately ---
    // Comment this out later when you tie it to real shooter values.
    double t = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double hoodAngleDeg = 35.0 + 15.0 * Math.sin(t);  // swings between 20 and 50 deg
    hoodLigament.setAngle(hoodAngleDeg);

    // Spin the flywheel ligament visually
    double flywheelAngleDeg = (t * 360.0) % 360.0;
    flywheelLigament.setAngle(flywheelAngleDeg);

    SmartDashboard.putNumber("Hood", hoodAngleDeg);
  }
}


