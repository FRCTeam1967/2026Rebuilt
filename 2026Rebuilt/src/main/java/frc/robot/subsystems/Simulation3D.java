// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Simulation3D extends SubsystemBase {
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(3, 3);

  private final LoggedMechanismLigament2d hoodLigament;
  private final LoggedMechanismLigament2d flywheelLigament;

  private Pose2d robotPose = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d());

  public Simulation3D() {
    // Root point where the shooter is mounted (2D drawing root)
    LoggedMechanismRoot2d root = mechanism.getRoot("ShooterRoot", 1.5, 0.2);

    hoodLigament =
        root.append(
            new LoggedMechanismLigament2d(
                "Hood",
                1.0,   // length
                35.0   // starting angle (degrees)
            ));

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
    double t = Timer.getFPGATimestamp();
    double x = 1.0 + 0.5 * Math.cos(t * 0.3);
    double y = 1.0 + 0.5 * Math.sin(t * 0.3);
    Rotation2d heading = Rotation2d.fromDegrees((t * 15.0) % 360.0);
    robotPose = new Pose2d(x, y, heading);

    Logger.recordOutput("Robot/Pose2d", robotPose);

    double hoodAngleDeg = 35.0 + 15.0 * Math.sin(t); 
    hoodLigament.setAngle(hoodAngleDeg);

    double flywheelAngleDeg = (t * 360.0) % 360.0;
    flywheelLigament.setAngle(flywheelAngleDeg);

    Logger.recordOutput("Shooter/Mechanism2d", mechanism);

    SmartDashboard.putNumber("Hood", hoodAngleDeg);
    ArrayList<Pose3d> componentPoses = mechanism.generate3dMechanism();

    Transform3d offset =
        new Transform3d(
            new Translation3d(0.30, 0.0, 0.55),   
            new Rotation3d(0.0, 0.0, 0.0)        
        );

    Pose3d[] posedArray =
        componentPoses.stream()
            .map(p -> p.transformBy(offset))
            .toArray(Pose3d[]::new);

    Logger.recordOutput("Shooter/Mechanism3d", posedArray);
  }
}



