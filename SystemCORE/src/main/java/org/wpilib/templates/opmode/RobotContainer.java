package org.wpilib.templates.opmode;
import java.util.Optional;

import org.wpilib.command2.Command;

import com.ctre.phoenix6.CANBus;

import dev.doglog.DogLog;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import first.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static final CANBus CANBus = null;

        //mechanism
        public final Climb climb = new Climb();

    //control
        public final CommandXboxController m_driverController = new CommandXboxController(0);
        public final CommandXboxController m_operatorController = new CommandXboxController(1);




    public RobotContainer() {
        configureBindings();
    }
    

    private void configureBindings() {

        //BUTTON BINDING: RUN CLIMB
            //m_operatorController.y().onTrue(new MoveClimbHalfwayDown(climb, -4)); 
            m_operatorController.povUp().onTrue(new MoveClimbUp(climb, -15)); 
            m_operatorController.povRight().onTrue(new MoveClimbHalfwayDown(climb, 14)); 
            m_operatorController.povDown().onTrue(new MoveClimbtoZero(climb, 15)); 
    }

    public Command getAutonomousCommand() {
        return null;
    }

}