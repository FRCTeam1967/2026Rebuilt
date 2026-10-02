// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Paths2Fly extends Command {

    private final Timer timer = new Timer();
    private Pose2d targetPose;
    private GoalEndState goalEndState;
    private final PathConstraints constraints;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final RobotConfig robotConfig;

    private PathPlannerPath currentPath;
    private PathPlannerTrajectory currentTrajectory;
    private EventScheduler eventScheduler = new EventScheduler();

    private Consumer<PathPlannerTrajectory> controller;


    private boolean beginGenerateJoinPath = false;
    private boolean skipUpdates = false;

    /**
     * Constructs a new base pathfinding command that will generate a path towards the given path.
     *
     * @param constraints the path constraints to use while pathfinding
     * @param poseSupplier a supplier for the robot's current pose
     * @param speedsSupplier a supplier for the robot's current robot relative speeds
     * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
     *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
     *     using a differential drive, they will be in L, R order.
     *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize
     *     your module states, you will need to reverse the feedforwards for modules that have been
     *     flipped
     * @param controller Path following controller that will be used to follow the path
     * @param robotConfig The robot configuration
     * @param shouldFlipPath Should the target path be flipped to the other side of the field? This
     *     will maintain a global blue alliance origin.
     * @param requirements the subsystems required by this command
     */
  
    public Paths2Fly(
            Pose2d targetPose,
            //PathConstraints constraints,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            Consumer<PathPlannerTrajectory> controller,
            RobotConfig robotConfig,
            BooleanSupplier shouldFlipPath,
            Subsystem requirements) {
        addRequirements(requirements);

        Pathfinding.ensureInitialized();

        Rotation2d targetRotation = Rotation2d.k180deg;
        double goalEndVel = 4.0;
        
        PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
        
        this.targetPose = new Pose2d(3.0, 4.0, targetRotation);
        this.goalEndState = new GoalEndState(goalEndVel, targetRotation);
        this.constraints = constraints;
        this.controller = controller;
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.robotConfig = robotConfig;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

