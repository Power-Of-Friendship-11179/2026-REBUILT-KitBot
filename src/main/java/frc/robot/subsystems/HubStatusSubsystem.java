// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

/**
 * This subsystem tracks the status of the hub and relays it to the drivers.
 * 
 * <p>
 * TODO Setup the hardware per this diagram
 * (https://docs.revrobotics.com/rev-crossover-products/blinkin/ion) and these
 * instructions
 * (https://docs.revrobotics.com/rev-crossover-products/blinkin/gs).
 * 
 * <p>
 * Some notes to go with those instructions.
 * <ul>
 * <li>We have a 5V strip. The 12V output lines will be left unattached.
 * <li>The PDH port where power is attached can have a 15 or 20 Amp breaker.
 * <li>The PWM port is set to 0 in the code. Change the Spark declaration below
 * if a different port would be better.
 * <li>Configure color 1 to be green.
 * <li>Configure color 2 to be red.
 * <li>Configure the number of LEDs in the strip.
 * <eul>
 */
public class HubStatusSubsystem extends SubsystemBase {
  private static final long TRANSITION_END_MILLIS = 10000;
  private static final long SHIFT_1_START_MILLIS = 10000;
  private static final long SHIFT_2_START_MILLIS = 35000;
  private static final long SHIFT_3_START_MILLIS = 60000;
  private static final long SHIFT_4_START_MILLIS = 85000;
  private static final long END_GAME_START_MILLIS = 110000;
  /** Array of critical times ordered for stepping through during teleop. */
  private static final long[] SHIFT_CHANGES = {
      0, // Make index match shift numbers
      SHIFT_1_START_MILLIS,
      SHIFT_2_START_MILLIS,
      SHIFT_3_START_MILLIS,
      SHIFT_4_START_MILLIS,
      END_GAME_START_MILLIS,
      Long.MAX_VALUE // Avoid inappropriate pulsing and simplify some checks.
  };
  private static final int END_GAME_SHIFT_INDEX = 5; // Keep in sync with above.
  private static final long PULSING_DURATION_MILLIS = 3000;
  private static final long PULSE_PERIOD_MILLIS = 500;
  private static final long AUTO_RESULT_DISPLAY_MILLIS = 2000;
  /**
   * When shifting from inactive to active, we want to start the shift, and its
   * associated pulsing early. This is to enable the drivers to start shooting as
   * early as possible. Once the auto winner is determined, our shift start times
   * in {@link #SHIFT_CHANGES} are adjusted earlier by this amount.
   */
  private static final long EARLY_SHIFT_START_MILLIS = 1500;

  private static final String DASHBOARD_FIELD = "Hub Active";
  private static final String RED_HEX = new Color(255, 0, 0).toHexString();
  private static final String GREEN_HEX = new Color(0, 255, 0).toHexString();
  private static final String BLACK_HEX = new Color(0, 0, 0).toHexString();
  private static final String WHITE_HEX = new Color(255, 255, 255).toHexString();

  private enum DriverFeedback {
    WON_AUTO(-0.99, WHITE_HEX), // Rainbow
    LOST_AUTO(-0.59, BLACK_HEX), // Fire
    ACTIVE(0.77, GREEN_HEX), // Green
    INACTIVE(0.61, RED_HEX), // Red
    NO_GAME_DATA(0.99, BLACK_HEX), // Black
    PULSE_ACTIVE(0.05, GREEN_HEX), // Heartbeat color 1
    PULSE_INACTIVE(0.25, RED_HEX); // Heartbeat color 2

    private final double ledValue;
    private final String dashboardColorHex;

    private DriverFeedback(final double ledValue, final String dashboardColorHex) {
      this.ledValue = ledValue;
      this.dashboardColorHex = dashboardColorHex;
    }
  }

  private long telopStartMillis = 0;
  private boolean initialized = false;
  private boolean haveGameData = false;
  private boolean weWonAuto = false;
  private long autoResultDisplayMillis = 0;
  private boolean weAreActive = false;
  private int nextShift = 1;
  private long nextPulseStart = Long.MAX_VALUE;
  private Spark leds = new Spark(0);
  private DriverFeedback currentFeedback = DriverFeedback.INACTIVE;

  /**
   * Create subsystem and bind robot mode commands.
   */
  public HubStatusSubsystem() {
    RobotModeTriggers.disabled().onTrue(runOnce(this::setInactive));
    RobotModeTriggers.autonomous().onTrue(runOnce(this::setActive));
    RobotModeTriggers.teleop().onTrue(run(this::startTeleop));
  }

  /**
   * Sets the driver feedback to hub inactive.
   */
  private void setInactive() {
    weAreActive = false;
    setDashboard(DriverFeedback.INACTIVE);
    setPattern(DriverFeedback.INACTIVE);
  }

  /**
   * Sets the driver feedback to hub active.
   */
  private void setActive() {
    weAreActive = true;
    setDashboard(DriverFeedback.ACTIVE);
    setPattern(DriverFeedback.ACTIVE);
  }

  /**
   * Set the dashboard color.
   * 
   * @param pattern desired driver feedback.
   */
  private void setDashboard(final DriverFeedback pattern) {
    SmartDashboard.putString(DASHBOARD_FIELD, pattern.dashboardColorHex);
  }

  /**
   * Set LED pattern.
   * 
   * @param pattern desired driver feedback.
   */
  private void setPattern(DriverFeedback pattern) {
    this.currentFeedback = pattern;
    leds.set(this.currentFeedback.ledValue);
  }

  /**
   * Returns a command that sets up to manage shift changes during teleop,
   * including handling the game data.
   * 
   * @return the teleop hub status initialization command.
   */
  private Command startTeleop() {
    return startRun(
        this::initializeTeleop,
        this::initializeFromGameData)
        .until(() -> initialized);
  }

  /**
   * The hub is set to active during transistion and the teleop start time is
   * recorded.
   */
  private void initializeTeleop() {
    setActive();
    telopStartMillis = System.currentTimeMillis();
  }

  /**
   * Attempts to read good game data until found, or transition ends (give up).
   * The winner of auto is recorded to enable handling of future transistions.
   * 
   * <p>
   * If we find valid game data, a separate command is started to handle the
   * transistions through the rest of teleop.
   */
  private void initializeFromGameData() {
    final String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() > 0) {
      final char autoWinner = gameData.charAt(0);
      if (autoWinner == 'R' || autoWinner == 'B') {
        haveGameData = true;
        initialized = true;
        autoResultDisplayMillis = (System.currentTimeMillis() - telopStartMillis) + AUTO_RESULT_DISPLAY_MILLIS;
        if (autoWinner == DriverStation.getAlliance().orElse(Alliance.Blue).name().charAt(0)) {
          weWonAuto = true;
          SHIFT_CHANGES[2] -= EARLY_SHIFT_START_MILLIS;
          SHIFT_CHANGES[4] -= EARLY_SHIFT_START_MILLIS;
          // No need to adjust 5/end game as we are active from 4 into end game.
          displayWonAuto();
        } else {
          weWonAuto = false;
          // No need to adjust 1 as we are active for auto into 1.
          SHIFT_CHANGES[3] -= EARLY_SHIFT_START_MILLIS;
          SHIFT_CHANGES[5] -= EARLY_SHIFT_START_MILLIS;
          displayLostAuto();
        }
        CommandScheduler.getInstance().schedule(
            run(this::runTeleopShifts)
                .until(() -> nextShift > END_GAME_SHIFT_INDEX));
      }
    }

    // Timeout after transition seconds without game data and show no indicator.
    if ((!haveGameData)
        && ((System.currentTimeMillis() - telopStartMillis) > TRANSITION_END_MILLIS)) {
      initialized = true;
      displayInvalidGameData();
    }
  }

  /**
   * Steps through the {@link #SHIFT_CHANGES} array handling the shift
   * transistions as they occur. Also handles pulsing toward the end of each
   * shift.
   */
  private void runTeleopShifts() {
    long elapsed = System.currentTimeMillis() - telopStartMillis;
    if (elapsed > autoResultDisplayMillis) {
      autoResultDisplayMillis = Long.MAX_VALUE;
      setActive();
    }
    if ((nextShift < SHIFT_CHANGES.length) && (elapsed > SHIFT_CHANGES[nextShift])) {
      if ((nextShift == 1) || (nextShift == 3)) {
        if (weWonAuto) {
          setInactive();
        } else {
          setActive();
        }
      } else if ((nextShift == 2) || (nextShift == 4)) {
        if (weWonAuto) {
          setActive();
        } else {
          setInactive();
        }
      } else { // this is end game
        setActive();
      }
      nextShift++;
      nextPulseStart = SHIFT_CHANGES[nextShift] - PULSING_DURATION_MILLIS;
    } else {
      manageEndStagePulsing(elapsed);
    }
  }

  /**
   * Feedback to the drivers that we won auto.
   */
  private void displayWonAuto() {
    setDashboard(DriverFeedback.WON_AUTO);
    setPattern(DriverFeedback.WON_AUTO);
  }

  /**
   * Feedback to the drivers that we lost auto.
   */
  private void displayLostAuto() {
    setDashboard(DriverFeedback.LOST_AUTO);
    setPattern(DriverFeedback.LOST_AUTO);
  }

  /**
   * Feedback to the drivers that we did not obtain valid game data.
   */
  private void displayInvalidGameData() {
    setDashboard(DriverFeedback.NO_GAME_DATA);
    setPattern(DriverFeedback.NO_GAME_DATA);
  }

  /**
   * Pulse at end of shift. Do not pulse end of transistion if we lost auto.
   * 
   * @param teleopElapsedMillis the teleop elapsed time in milliseconds.
   */
  private void manageEndStagePulsing(final long teleopElapsedMillis) {
    if ((teleopElapsedMillis > nextPulseStart) && !(nextShift == 1 && !weWonAuto)) {
      // Dashboard pulsing
      long pulsedSoFar = teleopElapsedMillis - nextPulseStart;
      if (((pulsedSoFar % PULSE_PERIOD_MILLIS) - (PULSE_PERIOD_MILLIS / 2)) < 0) {
        SmartDashboard.putString(DASHBOARD_FIELD, BLACK_HEX);
      } else {
        if (weAreActive) {
          setActive();
        } else {
          setInactive();
        }
      }
      // LED pulsing
      if ((currentFeedback != DriverFeedback.PULSE_ACTIVE) && (currentFeedback != DriverFeedback.PULSE_INACTIVE)) {
        if (weAreActive) {
          setPattern(DriverFeedback.PULSE_ACTIVE);
        } else {
          setPattern(DriverFeedback.PULSE_INACTIVE);
        }
      }
    }
  }
}
