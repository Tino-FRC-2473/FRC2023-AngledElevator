package frc.robot.systems;
// WPILib Imports



// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxLimitSwitch;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ElevatorWristFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	private enum FSMState {
		MOVING_IN,
		MOVING_OUT,
		ZEROING,
		IDLE
	}
	private static final double IN_SPEED = -0.1;
	private static final double OUT_SPEED = 0.1;
	private static final double ZEROING_SPEED = -0.01;
	private static final double PID_CONSTANT_WRIST_P = 0.00000001;
	private static final double PID_CONSTANT_WRIST_I = 0.00000001;
	private static final double PID_CONSTANT_WRIST_D = 0.00000001;
	private static final float MAX_UP_POWER = 0.2f;
	private static final float MAX_DOWN_POWER = -0.2f;
	private static final double WRIST_IN_ENCODER_ROTATIONS = -1;
	private static final double WRIST_OUT_ENCODER_ROTATIONS = 50;

	/* ======================== Private variables ======================== */
	private FSMState currentState;
	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax wristMotor;
	private SparkMaxPIDController pidControllerWrist;
	private SparkMaxLimitSwitch wristLimitSwitch;
	private double currentEncoder;
	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ElevatorWristFSM() {
		wristMotor = new CANSparkMax(HardwareMap.CAN_ID_WRIST_MOTOR,
				CANSparkMax.MotorType.kBrushless);
		wristLimitSwitch = wristMotor.getReverseLimitSwitch(
				SparkMaxLimitSwitch.Type.kNormallyClosed);
		wristLimitSwitch.enableLimitSwitch(true);
		pidControllerWrist = wristMotor.getPIDController();
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		pidControllerWrist.setP(PID_CONSTANT_WRIST_P);
		pidControllerWrist.setI(PID_CONSTANT_WRIST_I);
		pidControllerWrist.setD(PID_CONSTANT_WRIST_D);
		pidControllerWrist.setOutputRange(MAX_DOWN_POWER, MAX_UP_POWER);
		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.IDLE;
		currentEncoder = wristMotor.getEncoder().getPosition();
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		if (input == null) {
			return;
		}
		//Robot.getStringLog().append("spinning intake start in " + currentState.toString());
		/*if (input.isDisableUpdatedPressed()) {
			toggleUpdate = !toggleUpdate;
			SmartDashboard.putBoolean("Is update enabled", toggleUpdate);
		}*/
		if (wristLimitSwitch.isPressed()) {
			wristMotor.getEncoder().setPosition(0);
		}
		if (currentState != FSMState.IDLE) {
			currentEncoder = wristMotor.getEncoder().getPosition();
		}
		switch (currentState) {
			case MOVING_IN:
				handleMovingInState(input);
				break;
			case MOVING_OUT:
				handleMovingOutState(input);
				break;
			case ZEROING:
				handleZeroingState(input);
				break;
			case IDLE:
				handleIdleState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		SmartDashboard.putString("Current State", currentState.toString());
		SmartDashboard.putBoolean("Wrist Zeroed Limit Switch", wristLimitSwitch.isPressed());
		SmartDashboard.putNumber("Wrist Encoder", wristMotor.getEncoder().getPosition());
		currentState = nextState(input);
		//Robot.getStringLog().append("spinning intake ending");
		//Robot.getStringLog().append("Time taken for loop: " + timeTaken);
	}

	/*-------------------------NON HANDLER METHODS ------------------------- */
	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		if (input == null) {
			return FSMState.IDLE;
		}
		switch (currentState) {
			case IDLE:
				if (input.isWristOutButtonPressed() && !input.isWristInButtonPressed()
					&& !input.isWristZeroButtonPressed()) {
					//go to moving out state
					return FSMState.MOVING_OUT;
				} else if (input.isWristZeroButtonPressed()) {
					//go to zeroing state
					return FSMState.ZEROING;
				} else if (input.isWristInButtonPressed() && !input.isWristOutButtonPressed()
					&& !input.isWristZeroButtonPressed()) {
					//go to moving in state
					return FSMState.MOVING_IN;
				}
				//stay in idle state
				return FSMState.IDLE;
			case MOVING_OUT:
				if (!input.isWristOutButtonPressed()) {
					//go to idle state
					return FSMState.IDLE;
				}
				return FSMState.MOVING_OUT;
			case ZEROING:
				if (!input.isWristZeroButtonPressed()) {
					//go to idle state
					return FSMState.IDLE;
				}
				//stay in zeroing state
				return FSMState.ZEROING;
			case MOVING_IN:
				if (!input.isWristInButtonPressed()) {
					//go to idle state
					return FSMState.IDLE;
				}
				//stay in moving in state
				return FSMState.MOVING_IN;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in states.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 * the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		//PREVIOUS CODE: wristMotor.set(0);
		pidControllerWrist.setReference(currentEncoder, CANSparkMax.ControlType.kPosition);
	}
	private void handleMovingInState(TeleopInput input) {
		pidControllerWrist.setReference(WRIST_IN_ENCODER_ROTATIONS,
			CANSparkMax.ControlType.kPosition);
	}
	private void handleMovingOutState(TeleopInput input) {
		pidControllerWrist.setReference(WRIST_OUT_ENCODER_ROTATIONS,
			CANSparkMax.ControlType.kPosition);
	}
	private void handleZeroingState(TeleopInput input) {
		pidControllerWrist.setReference(ZEROING_SPEED, CANSparkMax.ControlType.kDutyCycle);
	}
}
