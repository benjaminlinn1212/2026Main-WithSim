package frc.robot.subsystems.orchestra;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Used when superstructure motors are enabled
import frc.robot.Constants.OrchestraConstants;
import java.util.ArrayList;
import java.util.List;

/**
 * Manages a CTRE Phoenix 6 Orchestra instance for playing .chrp music files through Kraken / Talon
 * FX motors. Creates lightweight TalonFX references to the same CAN IDs used by the robot's
 * subsystems — Orchestra takes over MusicTone control while playing and releases the motors when
 * stopped.
 *
 * <p>Place your .chrp file in {@code src/main/deploy/} — it will be automatically deployed to the
 * roboRIO. Convert MIDI → CHRP using Phoenix Tuner X's CHRP Converter tool.
 *
 * <p>Usage:
 *
 * <pre>
 *   OrchestraManager orchestra = new OrchestraManager();
 *   autoChooser.addOption("Play Music", orchestra.playMusicCommand());
 * </pre>
 */
public class OrchestraManager extends SubsystemBase {

  private final Orchestra orchestra = new Orchestra();
  private final List<TalonFX> instruments = new ArrayList<>();
  private boolean musicLoaded = false;

  /** Creates the OrchestraManager, adds all configured motors as instruments, and loads music. */
  public OrchestraManager() {
    // ── Create TalonFX references for each motor we want in the orchestra ──
    // These point to the SAME physical motors as the subsystem IO classes.
    // Orchestra will send MusicTone control requests while playing.

    // Superstructure motors (all on SUPERSTRUCTURE_CAN_BUS) — uncomment to include
    // CANBus superstructureBus = Constants.SUPERSTRUCTURE_CAN_BUS;
    // addMotor(Constants.IntakeConstants.UPPER_MOTOR_CAN_ID, superstructureBus); // Intake upper
    // addMotor(Constants.IntakeConstants.LOWER_MOTOR_CAN_ID, superstructureBus); // Intake lower
    // addMotor(Constants.IntakePivotConstants.MOTOR_CAN_ID, superstructureBus); // Intake pivot
    // addMotor(Constants.TurretConstants.MOTOR_CAN_ID, superstructureBus); // Turret
    // addMotor(Constants.ConveyorConstants.MOTOR_CAN_ID, superstructureBus); // Conveyor
    // addMotor(Constants.IndexerConstants.LEADER_MOTOR_CAN_ID, superstructureBus); // Indexer
    // leader
    // addMotor(Constants.IndexerConstants.FOLLOWER_MOTOR_CAN_ID, superstructureBus); // Indexer
    // follow
    // addMotor(Constants.ShooterConstants.MOTOR_CAN_ID, superstructureBus); // Shooter
    // addMotor(Constants.HoodConstants.MOTOR_CAN_ID, superstructureBus); // Hood

    // Climb motors (also on SUPERSTRUCTURE_CAN_BUS) — uncomment to include
    // addMotor(Constants.ClimbConstants.RIGHT_FRONT_MOTOR_CAN_ID, superstructureBus);
    // addMotor(Constants.ClimbConstants.RIGHT_BACK_MOTOR_CAN_ID, superstructureBus);
    // addMotor(Constants.ClimbConstants.LEFT_FRONT_MOTOR_CAN_ID, superstructureBus);
    // addMotor(Constants.ClimbConstants.LEFT_BACK_MOTOR_CAN_ID, superstructureBus);

    // Drivetrain motors (on Drivetrain CAN bus)
    CANBus drivetrainBus = frc.robot.generated.TunerConstants.kCANBus;
    addMotor(21, drivetrainBus); // FL drive
    addMotor(24, drivetrainBus); // FR drive
    addMotor(22, drivetrainBus); // BL drive
    addMotor(23, drivetrainBus); // BR drive
    addMotor(31, drivetrainBus); // FL steer
    addMotor(34, drivetrainBus); // FR steer
    addMotor(32, drivetrainBus); // BL steer
    addMotor(33, drivetrainBus); // BR steer

    // ── Add all instruments to the Orchestra ──
    // Motors are distributed across tracks using modulo wrapping.
    // With NUM_TRACKS=1: all 8 motors play the same part (loudest).
    // With NUM_TRACKS=2: motors 0,2,4,6 → track 0, motors 1,3,5,7 → track 1.
    // With NUM_TRACKS=4: 2 motors per track, etc.
    int numTracks = OrchestraConstants.NUM_TRACKS;
    for (int i = 0; i < instruments.size(); i++) {
      int track = i % numTracks;
      StatusCode status = orchestra.addInstrument(instruments.get(i), track);
      if (!status.isOK()) {
        System.err.println(
            "[Orchestra] Failed to add instrument (CAN ID "
                + instruments.get(i).getDeviceID()
                + ") to track "
                + track
                + ": "
                + status);
      }
    }

    // ── Enable playback while robot is disabled (optional) ──
    if (OrchestraConstants.ALLOW_MUSIC_DURING_DISABLE) {
      enableMusicDuringDisable();
    }

    // ── Load the chirp file ──
    loadTrack(OrchestraConstants.CHRP_FILE);

    System.out.println(
        "[Orchestra] Initialized with "
            + instruments.size()
            + " instruments, track: "
            + OrchestraConstants.CHRP_FILE);
  }

  /**
   * Creates a TalonFX reference and adds it to the instrument list.
   *
   * @param canId the CAN ID of the motor
   * @param canBus the CAN bus
   */
  private void addMotor(int canId, CANBus canBus) {
    TalonFX motor = new TalonFX(canId, canBus);
    instruments.add(motor);
  }

  /**
   * Enables the AllowMusicDurDisable config on all instruments so music can play while the robot is
   * disabled.
   */
  private void enableMusicDuringDisable() {
    for (TalonFX motor : instruments) {
      var config = new AudioConfigs();
      config.AllowMusicDurDisable = true;
      motor.getConfigurator().apply(config);
    }
  }

  /**
   * Loads a .chrp file into the Orchestra.
   *
   * @param filename the filename (e.g. "song.chrp") — files in src/main/deploy are auto-deployed.
   */
  public void loadTrack(String filename) {
    StatusCode status = orchestra.loadMusic(filename);
    musicLoaded = status.isOK();
    if (!musicLoaded) {
      System.err.println("[Orchestra] Failed to load music '" + filename + "': " + status);
    } else {
      System.out.println("[Orchestra] Loaded track: " + filename);
    }
  }

  /** Starts playback. Only needs to be called once — music continues until paused or stopped. */
  public void play() {
    if (musicLoaded) {
      orchestra.play();
    }
  }

  /** Pauses playback — can be resumed with {@link #play()}. */
  public void pause() {
    orchestra.pause();
  }

  /** Stops playback and resets to the beginning of the track. */
  public void stop() {
    orchestra.stop();
  }

  /**
   * @return true if the orchestra is currently playing music.
   */
  public boolean isPlaying() {
    return orchestra.isPlaying();
  }

  /**
   * @return the current playback time in seconds.
   */
  public double getCurrentTime() {
    return orchestra.getCurrentTime();
  }

  // ──────────────────────────── Command Factories ────────────────────────────

  /**
   * Returns a Command that plays music for the entire autonomous period. The music starts on
   * initialize, runs until the command is interrupted or the track finishes, then stops the
   * orchestra so normal motor control resumes.
   *
   * <p>This command requires this subsystem to prevent other orchestra commands from conflicting.
   */
  public Command playMusicCommand() {
    return Commands.startEnd(this::play, this::stop, this)
        .withName("PlayMusic")
        .until(() -> !isPlaying() && getCurrentTime() > 0);
  }

  /**
   * Returns a Command that plays music indefinitely until interrupted. Useful for teleop triggers
   * or testing.
   */
  public Command playUntilInterruptedCommand() {
    return Commands.startEnd(this::play, this::stop, this).withName("PlayMusicUntilInterrupted");
  }

  /**
   * Returns a Command that loads a new track and then plays it.
   *
   * @param filename the .chrp filename to load and play
   */
  public Command loadAndPlayCommand(String filename) {
    return Commands.runOnce(() -> loadTrack(filename), this)
        .andThen(playMusicCommand())
        .withName("LoadAndPlay(" + filename + ")");
  }
}
