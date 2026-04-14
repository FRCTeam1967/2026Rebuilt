package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/*
 * Autonomous: 20 seconds (Both Hubs Active).
 * (Disabled period)
 * Transition: 10 seconds (Auto-to-Teleop).
 * Teleop Shifts: Four 25-second shifts where alliance hubs alternate between active and inactive.
 * Endgame: Final 30 seconds (Both Hubs Active)
 * Shift Order: The alliance that loses the autonomous bonus starts with their hub inactive in the first shift.
 * 
 */

public class HubTracker {
    private class Shift {
        public double start;
        public double end;

        public Shift(double start, double end) {
            this.start = start;
            this.end = end;
        }
    }

    public static final double TRANSITION_END = 10.0;
    public static final double FIRST_SHIFT_END = 35.0;
    public static final double SECOND_SHIFT_END = 60.0;
    public static final double THIRD_SHIFT_END = 85.0;
    public static final double FOURTH_SHIFT_END = 110.0;
    public static final double TELEOP_END = 140.0;

    private static String currentGameData = "";
    private Timer matchTimer = new Timer();

    // All shifts (not used)
    // private final Shift allShifts[] = new Shift[] {
    //     new Shift(0, TRANSITION_END),
    //     new Shift(TRANSITION_END, FIRST_SHIFT_END),
    //     new Shift(FIRST_SHIFT_END, SECOND_SHIFT_END),
    //     new Shift(SECOND_SHIFT_END, THIRD_SHIFT_END),
    //     new Shift(THIRD_SHIFT_END, FOURTH_SHIFT_END),
    //     new Shift(FOURTH_SHIFT_END, TELEOP_END),
    // };

    // Active times for the auto-winning alliance: transition, second shift, fourth shift + endgame
    private final Shift winningActiveShifts[] = new Shift[] {
        new Shift(0, TRANSITION_END), 
        new Shift(FIRST_SHIFT_END, SECOND_SHIFT_END), // 2nd shift
        new Shift(THIRD_SHIFT_END, TELEOP_END), // 4th shift + endgame
    };

    private final Shift winningInactiveShifts[] = new Shift[] {
        new Shift(TRANSITION_END, FIRST_SHIFT_END), // 1st shift
        new Shift(SECOND_SHIFT_END, THIRD_SHIFT_END), // 3rd shift
    };

    // Active times for the auto-losing alliance: transition + first shift, third shift, endgame
    private final Shift losingActiveShifts[] = new Shift[] {
        new Shift(0, FIRST_SHIFT_END), // transition + 1st shift
        new Shift(SECOND_SHIFT_END, THIRD_SHIFT_END), // 3rd shift
        new Shift(FOURTH_SHIFT_END, TELEOP_END), // Endgame
    };

    private final Shift losingInactiveShifts[] = new Shift[] {
        new Shift(FIRST_SHIFT_END, SECOND_SHIFT_END), // 2nd shift
        new Shift(THIRD_SHIFT_END, FOURTH_SHIFT_END), // 4th shift
    };

    /**
     * Determine who won auto
     * @return winning alliance, or NULL if not yet known
     */
    public Alliance autoWinningAlliance() {
        if (currentGameData.length() == 0) {
            currentGameData = DriverStation.getGameSpecificMessage();
            if (currentGameData.length() == 0) {
                return null;
            }
        }

        DogLog.log("HubTracker/winningAlliance", currentGameData);
        return currentGameData.charAt(0) == 'R' ? Alliance.Red : Alliance.Blue;
    }

    /**
     * Determine if our alliance won auto
     * @return true if we're known to have won auto, false otherwise
     */
    public boolean didWeWinAuto() {
        Alliance winningAlliance = autoWinningAlliance();
        Alliance teamAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (winningAlliance != null && winningAlliance == teamAlliance) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Determine the time until the next time our hub is active
     * @return time until hub is active, or 0 if it's already active
     */
    public double timeUntilActive() {
        double currentTime = matchTimer.get();
        Shift inactiveShifts[] = didWeWinAuto() ? winningInactiveShifts : losingInactiveShifts;
        for (Shift shift : inactiveShifts) {
            if (currentTime >= shift.start && currentTime < shift.end) {
                // Currently inactive
                return shift.end - currentTime;
            }
        }

        // We're currently active
        return 0.0;
    }

    /**
     * Determine the time until the hub becomes inactive
     * @return time until hub is inactive, or 0 if it's already inactive
     */
    public double timeUntilInactive() {
        double currentTime = matchTimer.get();
        Shift activeShifts[] = didWeWinAuto() ? winningActiveShifts : losingActiveShifts;
        for (Shift shift : activeShifts) {
            if (currentTime >= shift.start && currentTime < shift.end) {
                // Found where we are
                return shift.end - currentTime;
            }
        }

        // We're not in a current active period
        return 0.0;
    }

    /**
     * Must be called when teleop starts. If the robot reboots during teleop, data will be incorrect, but we
     * have bigger problems to worry about.
     */
    public void startHubTracking() {
        matchTimer.reset();
        matchTimer.start();
    }

}
