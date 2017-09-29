/**
 ******************************************************************************
 * @addtogroup Modules Modules
 * @{
 * @addtogroup ActuatorModule Actuator Module
 * @{
 *
 * @file       actuator.c
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2015-2016
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013-2016
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Actuator module. Drives the actuators (servos, motors etc).
 * @brief      Take the values in @ref ActuatorDesired and mix to set the outputs
 *
 * This module ultimately controls the outputs.  The values from @ref ActuatorDesired
 * are combined based on the values in @ref MixerSettings and then scaled by the
 * values in @ref ActuatorSettings to create the output PWM times.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

#include <math.h>

#include "openpilot.h"
#include "actuatorsettings.h"
#include "systemsettings.h"
#include "actuatordesired.h"
#include "actuatorcommand.h"
#include "flightstatus.h"
#include "mixersettings.h"
#include "cameradesired.h"
#include "manualcontrolcommand.h"
#include "pios_thread.h"
#include "pios_queue.h"
#include "misc_math.h"

#if defined(PIOS_INCLUDE_ESCTELEMETRY)
#include "pios_delay.h"
#include "pios_esctelemetry.h"
#include "actuatortelemetry.h"
#endif

// Private constants
#define MAX_QUEUE_SIZE 2

#if defined(PIOS_ACTUATOR_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_ACTUATOR_STACK_SIZE
#else
#define STACK_SIZE_BYTES 1336
#endif

#define TASK_PRIORITY PIOS_THREAD_PRIO_HIGHEST
#define FAILSAFE_TIMEOUT_MS 100

#ifndef MAX_MIX_ACTUATORS
#define MAX_MIX_ACTUATORS ACTUATORCOMMAND_CHANNEL_NUMELEM
#endif

DONT_BUILD_IF(ACTUATORSETTINGS_TIMERUPDATEFREQ_NUMELEM > PIOS_SERVO_MAX_BANKS, TooManyServoBanks);
DONT_BUILD_IF(MAX_MIX_ACTUATORS > ACTUATORCOMMAND_CHANNEL_NUMELEM, TooManyMixers);
DONT_BUILD_IF((MIXERSETTINGS_MIXER1VECTOR_NUMELEM - MIXERSETTINGS_MIXER1VECTOR_ACCESSORY0) < MANUALCONTROLCOMMAND_ACCESSORY_NUMELEM, AccessoryMismatch);

#define MIXER_SCALE 128

// Private types

// Private variables
static struct pios_queue *queue;
static struct pios_thread *taskHandle;

// used to inform the actuator thread that actuator / mixer settings are updated
// set true to ensure they're fetched on first run
static volatile bool flight_status_updated = true;
static volatile bool manual_control_cmd_updated = true;
static volatile bool actuator_settings_updated = true;
static volatile bool mixer_settings_updated = true;

static MixerSettingsMixer1TypeOptions types_mixer[MAX_MIX_ACTUATORS];

#if defined(PIOS_INCLUDE_ESCTELEMETRY)

#define ESC_TELEM_CYCLE_uS		200000	/* Poll ESCs every 200ms. */
#define ESC_TELEM_TIMEOUT_uS	5000	/* Transmission is supposed to take 900us. Timeout at 5ms. */

struct telemetry_state {

	uint32_t cycle_start;		/* Timestamp of when the current round of requests started. */
	uint32_t req_start;			/* Timestamp of ongoing request on single ESC. */
	uint8_t push;				/* Flag whether to push the UAVO. */
	uint8_t servo;				/* Servo counter. */
	uint8_t active;				/* Active servo number. */

	uint8_t ok;					/* Successful response counter, flag. */
	float voltage;				/* Accumulated voltages, will be divided by value in 'ok'. */
	float consumed;				/* Accumulated reported consumption. */

	ActuatorTelemetryData telemetry;

};

static struct telemetry_state *tlm;

#endif

/* In the mixer, a row consists of values for one output actuator.
 * A column consists of values for scaling one axis's desired command.
 */

static float motor_mixer[MAX_MIX_ACTUATORS * MIXERSETTINGS_MIXER1VECTOR_NUMELEM];

/* These are various settings objects used throughout the actuator code */
static ActuatorSettingsData actuatorSettings;
static SystemSettingsAirframeTypeOptions airframe_type;

static float curve1[MIXERSETTINGS_THROTTLECURVE1_NUMELEM];
static float curve2[MIXERSETTINGS_THROTTLECURVE2_NUMELEM];

static MixerSettingsCurve2SourceOptions curve2_src;

// Private functions
static void actuator_task(void* parameters);

static float scale_channel(float value, int idx);
static void set_failsafe();

static float throt_curve(const float input, const float *curve,
		uint8_t num_points);
static float collective_curve(const float input, const float *curve,
		uint8_t num_points);

volatile enum actuator_interlock actuator_interlock = ACTUATOR_INTERLOCK_OK;

/**
 * @brief Module initialization
 * @return 0
 */
int32_t ActuatorStart()
{
	// Watchdog must be registered before starting task
	PIOS_WDG_RegisterFlag(PIOS_WDG_ACTUATOR);

	// Start main task
	taskHandle = PIOS_Thread_Create(actuator_task, "Actuator", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
	TaskMonitorAdd(TASKINFO_RUNNING_ACTUATOR, taskHandle);

	return 0;
}

/**
 * @brief Module initialization
 * @return 0
 */
int32_t ActuatorInitialize()
{
	// Register for notification of changes to ActuatorSettings
	if (ActuatorSettingsInitialize()  == -1) {
		return -1;
	}
	ActuatorSettingsConnectCallbackCtx(UAVObjCbSetFlag, &actuator_settings_updated);

	// Register for notification of changes to MixerSettings
	if (MixerSettingsInitialize()  == -1) {
		return -1;
	}
	MixerSettingsConnectCallbackCtx(UAVObjCbSetFlag, &mixer_settings_updated);

	// Listen for ActuatorDesired updates (Primary input to this module)
	if (ActuatorDesiredInitialize()  == -1) {
		return -1;
	}

	queue = PIOS_Queue_Create(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));
	ActuatorDesiredConnectQueue(queue);

	// Primary output of this module
	if (ActuatorCommandInitialize() == -1) {
		return -1;
	}

#if defined(MIXERSTATUS_DIAGNOSTICS)
	// UAVO only used for inspecting the internal status of the mixer during debug
	if (MixerStatusInitialize()  == -1) {
		return -1;
	}
#endif

#if defined(PIOS_INCLUDE_ESCTELEMETRY)
	if (ActuatorTelemetryInitialize() == -1) {
		return -1;
	}
#endif

	return 0;
}
MODULE_HIPRI_INITCALL(ActuatorInitialize, ActuatorStart);

static float get_curve2_source(ActuatorDesiredData *desired, SystemSettingsAirframeTypeOptions airframe_type, MixerSettingsCurve2SourceOptions source)
{
	float tmp;

	switch (source) {
	case MIXERSETTINGS_CURVE2SOURCE_THROTTLE:
		if(airframe_type == SYSTEMSETTINGS_AIRFRAMETYPE_HELICP)
		{
			ManualControlCommandThrottleGet(&tmp);
			return tmp;
		}
		return desired->Thrust;
		break;
	case MIXERSETTINGS_CURVE2SOURCE_ROLL:
		return desired->Roll;
		break;
	case MIXERSETTINGS_CURVE2SOURCE_PITCH:
		return desired->Pitch;
		break;
	case MIXERSETTINGS_CURVE2SOURCE_YAW:
		return desired->Yaw;
		break;
	case MIXERSETTINGS_CURVE2SOURCE_COLLECTIVE:
		if (airframe_type == SYSTEMSETTINGS_AIRFRAMETYPE_HELICP)
		{
			return desired->Thrust;
		}
		ManualControlCommandCollectiveGet(&tmp);
		return tmp;
		break;
	case MIXERSETTINGS_CURVE2SOURCE_ACCESSORY0:
	case MIXERSETTINGS_CURVE2SOURCE_ACCESSORY1:
	case MIXERSETTINGS_CURVE2SOURCE_ACCESSORY2:
		(void) 0;

		int idx = source - MIXERSETTINGS_CURVE2SOURCE_ACCESSORY0;

		if (idx < 0) {
			return 0;
		}

		if (idx >= MANUALCONTROLCOMMAND_ACCESSORY_NUMELEM) {
			return 0;
		}

		float accessories[MANUALCONTROLCOMMAND_ACCESSORY_NUMELEM];

		ManualControlCommandAccessoryGet(accessories);

		return accessories[idx];
		break;
	}

	/* Can't get here */
	return 0;
}

static void compute_one_mixer(int mixnum,
		int16_t (*vals)[MIXERSETTINGS_MIXER1VECTOR_NUMELEM],
		MixerSettingsMixer1TypeOptions type)
{
	types_mixer[mixnum] = type;

	mixnum *= MIXERSETTINGS_MIXER1VECTOR_NUMELEM;

	if ((type != MIXERSETTINGS_MIXER1TYPE_SERVO) &&
			(type != MIXERSETTINGS_MIXER1TYPE_MOTOR)) {
		for (int i = 0; i < MIXERSETTINGS_MIXER1VECTOR_NUMELEM; i++) {
			// Ensure unused types are zero-filled
			motor_mixer[mixnum+i] = 0;
		}
	} else {
		for (int i = 0; i < MIXERSETTINGS_MIXER1VECTOR_NUMELEM; i++) {
			motor_mixer[mixnum+i] = (*vals)[i] * (1.0f / MIXER_SCALE);
		}
	}
}

/* Here be dragons */
#define compute_one_token_paste(b) compute_one_mixer(b-1, &mixerSettings.Mixer ## b ## Vector, mixerSettings.Mixer ## b ## Type)

static void compute_mixer()
{
	MixerSettingsData mixerSettings;

	MixerSettingsGet(&mixerSettings);

#if MAX_MIX_ACTUATORS > 0
	compute_one_token_paste(1);
#endif
#if MAX_MIX_ACTUATORS > 1
	compute_one_token_paste(2);
#endif
#if MAX_MIX_ACTUATORS > 2
	compute_one_token_paste(3);
#endif
#if MAX_MIX_ACTUATORS > 3
	compute_one_token_paste(4);
#endif
#if MAX_MIX_ACTUATORS > 4
	compute_one_token_paste(5);
#endif
#if MAX_MIX_ACTUATORS > 5
	compute_one_token_paste(6);
#endif
#if MAX_MIX_ACTUATORS > 6
	compute_one_token_paste(7);
#endif
#if MAX_MIX_ACTUATORS > 7
	compute_one_token_paste(8);
#endif
#if MAX_MIX_ACTUATORS > 8
	compute_one_token_paste(9);
#endif
#if MAX_MIX_ACTUATORS > 9
	compute_one_token_paste(10);
#endif
}

static void fill_desired_vector(
		ActuatorDesiredData *desired,
		float val1, float val2,
		float (*cmd_vector)[MIXERSETTINGS_MIXER1VECTOR_NUMELEM])
{
	(*cmd_vector)[MIXERSETTINGS_MIXER1VECTOR_THROTTLECURVE1] = val1;
	(*cmd_vector)[MIXERSETTINGS_MIXER1VECTOR_THROTTLECURVE2] = val2;
	(*cmd_vector)[MIXERSETTINGS_MIXER1VECTOR_ROLL] = desired->Roll;
	(*cmd_vector)[MIXERSETTINGS_MIXER1VECTOR_PITCH] = desired->Pitch;
	(*cmd_vector)[MIXERSETTINGS_MIXER1VECTOR_YAW] = desired->Yaw;

	/* Accessory0..Accessory2 are filled in when ManualControl changes
	 * in normalize_input_data
	 */
}

static void post_process_scale_and_commit(float *motor_vect,
		float *desired_vect, float dT,
		bool armed, bool spin_while_armed, bool stabilize_now,
		float *maxpoweradd_bucket)
{
	float min_chan = INFINITY;
	float max_chan = -INFINITY;
	float neg_clip = 0;
	int num_motors = 0;
	ActuatorCommandData command;

	const float hangtime_leakybucket_timeconstant = 0.3f;

	/* Hangtime maximum power add is now a "leaky bucket" system, ensuring
	 * that the average added power in the long term is the configured value
	 * but allowing higher values briefly.
	 *
	 * The intention is to allow more aggressive maneuvers than hangtime
	 * previously did, while still providing similar safety properties.
	 * A secondary motivation is to prevent tumbling when throttle is
	 * chopped during fast-forward-flight and more than hangtime power
	 * levels are needed for stabilization (because of aerodynamic forces).
	 *
	 * The "maximum" stored is based on recent throttle history-- it decays
	 * with time; at high throttle it corresponds to 300ms of the current
	 * power; at lower throttle it corresponds to 300ms of double the
	 * configured value.
	 */
	float maxpoweradd_softlimit = MAX(
			2 * actuatorSettings.LowPowerStabilizationMaxPowerAdd,
			desired_vect[MIXERSETTINGS_MIXER1VECTOR_THROTTLECURVE1])
		* hangtime_leakybucket_timeconstant;

	/* If we're under the limit, add this tick's hangtime power allotment */
	if (*maxpoweradd_bucket < maxpoweradd_softlimit) {
		*maxpoweradd_bucket += actuatorSettings.LowPowerStabilizationMaxPowerAdd * dT;
	} else {
		/* Otherwise, decay towards the current limit on a 300ms
		 * time constant.
		 */
		float alpha = dT / (dT + hangtime_leakybucket_timeconstant);

		*maxpoweradd_bucket = alpha * maxpoweradd_softlimit +
			(1-alpha) * (*maxpoweradd_bucket);
	}

	/* The maximum power add is what would spend the current allotment in
	 * 300ms.  In other words, in the absence of recent high-throttle,
	 * start from double the hangtime configured percentage and decay on
	 * a 300ms time constant IF IT IS ACTUALLY USED.
	 *
	 * This is separate from the above decay, so we could actually be
	 * decaying twice as fast if both are in play.
	 */
	float maxpoweradd = (*maxpoweradd_bucket) / hangtime_leakybucket_timeconstant;

	for (int ct = 0; ct < MAX_MIX_ACTUATORS; ct++) {
		switch (types_mixer[ct]) {
			case MIXERSETTINGS_MIXER1TYPE_DISABLED:
				// Set to minimum if disabled.
				// This is not the same as saying
				// PWM pulse = 0 us
				motor_vect[ct] = -1;
				break;

			case MIXERSETTINGS_MIXER1TYPE_SERVO:
				break;

			case MIXERSETTINGS_MIXER1TYPE_MOTOR:
				min_chan = fminf(min_chan, motor_vect[ct]);
				max_chan = fmaxf(max_chan, motor_vect[ct]);

				if (motor_vect[ct] < 0.0f) {
					neg_clip += motor_vect[ct];
				}

				num_motors++;
				break;
			case MIXERSETTINGS_MIXER1TYPE_CAMERAPITCH:
				if (CameraDesiredHandle()) {
					CameraDesiredPitchGet(
							&motor_vect[ct]);
				} else {
					motor_vect[ct] = -1;
				}
				break;
			case MIXERSETTINGS_MIXER1TYPE_CAMERAROLL:
				if (CameraDesiredHandle()) {
					CameraDesiredRollGet(
							&motor_vect[ct]);
				} else {
					motor_vect[ct] = -1;
				}
				break;
			case MIXERSETTINGS_MIXER1TYPE_CAMERAYAW:
				if (CameraDesiredHandle()) {
					CameraDesiredRollGet(
							&motor_vect[ct]);
				} else {
					motor_vect[ct] = -1;
				}
				break;
			default:
				set_failsafe();
				PIOS_Assert(0);
		}
	}

	float gain = 1.0f;
	float offset = 0.0f;

	/* This is a little dubious.  Scale down command ranges to
	 * fit.  It may cause some cross-axis coupling, though
	 * generally less than if we were to actually let it clip.
	 */
	if ((max_chan - min_chan) > 1.0f) {
		gain = 1.0f / (max_chan - min_chan);

		max_chan *= gain;
		min_chan *= gain;
	}

	/* Sacrifice throttle because of clipping */
	if (max_chan > 1.0f) {
		offset = 1.0f - max_chan;
	} else if (min_chan < 0.0f) {
		/* Low-side clip management-- how much power are we
		 * willing to add??? */

		neg_clip /= num_motors;

		/* neg_clip is now the amount of throttle "already added." by
		 * clipping...
		 *
		 * Find the "highest possible value" of offset.
		 * if neg_clip is -15%, and maxpoweradd is 10%, we need to add
		 * -5% to all motors.
		 * if neg_clip is 5%, and maxpoweradd is 10%, we can add up to
		 * 5% to all motors to further fix clipping.
		 */
		offset = neg_clip + maxpoweradd;

		/* Add the lesser of--
		 * A) the amount the lowest channel is out of range.
		 * B) the above calculated offset.
		 */
		offset = MIN(-min_chan, offset);

		/* The amount actually added is the above offset, plus the
		 * amount that came from negative clipping.  (It's negative
		 * though, so subtract instead of add).  Spend this from
		 * the leaky bucket. 
		 */
		*maxpoweradd_bucket -= (offset - neg_clip) * dT;
	}

	for (int ct = 0; ct < MAX_MIX_ACTUATORS; ct++) {
		// Motors have additional protection for when to be on
		if (types_mixer[ct] == MIXERSETTINGS_MIXER1TYPE_MOTOR) {
			if (!armed) {
				motor_vect[ct] = -1;  //force min throttle
			} else if (!stabilize_now) {
				if (!spin_while_armed) {
					motor_vect[ct] = -1;
				} else {
					motor_vect[ct] = 0;
				}
			} else {
				motor_vect[ct] = motor_vect[ct] * gain + offset;

				if (motor_vect[ct] > 0) {
					// Apply curve fitting, mapping the input to the propeller output.
					motor_vect[ct] = powapprox(motor_vect[ct], actuatorSettings.MotorInputOutputCurveFit);
				} else {
					motor_vect[ct] = 0;
				}
			}
		}

		command.Channel[ct] = scale_channel(motor_vect[ct], ct);
	}

	// Store update time
	command.UpdateTime = 1000.0f*dT;

	ActuatorCommandMaxUpdateTimeGet(&command.MaxUpdateTime);

	if (command.UpdateTime > command.MaxUpdateTime)
		command.MaxUpdateTime = 1000.0f*dT;

	// Update output object
	if (!ActuatorCommandReadOnly()) {
		ActuatorCommandSet(&command);
	} else {
		// it's read only during servo configuration--
		// so GCS takes precedence.
		ActuatorCommandGet(&command);
	}

#if defined(PIOS_INCLUDE_ESCTELEMETRY)
	if (PIOS_ESCTelemetry_IsAvailable()) {

		/* Process telemetry request stuff before servo settings. */

		if (!tlm) {
			tlm = PIOS_malloc_no_dma(sizeof(*tlm));
			if (!tlm)
				PIOS_Assert(0);

			tlm->servo = 0xFF;
			tlm->active = 0xFF;

			ActuatorTelemetryGet(&tlm->telemetry);
		}

		/* Just cycle naively through all servos. */
		if (tlm->servo >= MAX_MIX_ACTUATORS && PIOS_DELAY_GetuSSince(tlm->cycle_start) > ESC_TELEM_CYCLE_uS) {
			tlm->servo = 0;
			tlm->ok = 0;
			tlm->cycle_start = PIOS_DELAY_GetuS();
		}

		if ((tlm->active == 0xFF) && (tlm->servo < MAX_MIX_ACTUATORS)) {
			/* If active servo is 0xFF, no request is active, either because we're starting over,
			   the servo doesn't exists, or isn't configured to DShot. */

			tlm->active = tlm->servo;
			tlm->servo++;

			tlm->req_start = PIOS_DELAY_GetuS();

			/* We don't care about ongoing requests, because we shouldn't even be here, if one is. */
			if (PIOS_Servo_RequestTelemetry(tlm->active) < -1)
				tlm->active = 0xFF;

			/* If this is the last servo being polled, push the UAVO, once the request finishes. */
			if (tlm->servo >= MAX_MIX_ACTUATORS)
				tlm->push = 1;

		} else if (tlm->active < 0xFF) {
			if (PIOS_ESCTelemetry_DataAvailable()) {
				/* Parse and stick info UAVO. */
				struct pios_esctelemetry_info t;
				PIOS_ESCTelemetry_Get(&t);

				tlm->voltage += t.voltage;
				tlm->consumed += t.mAh;
				tlm->telemetry.Temperature[tlm->active] = t.temperature;
				tlm->telemetry.Current[tlm->active] = t.current * 0.01f;
				tlm->telemetry.eRPM[tlm->active] = t.rpm;
				tlm->ok++;

				tlm->active = 0xFF;
			} else if (PIOS_DELAY_GetuSSince(tlm->req_start) > ESC_TELEM_TIMEOUT_uS) {
				/* Just hold values on a timeout. */
				tlm->active = 0xFF;
			}
		}

		if (tlm && tlm->push && tlm->active == 0xFF && tlm->ok) {
			tlm->telemetry.Voltage = tlm->voltage / (float)tlm->ok;
			/* Only replace the Consumed value in the UAVO, if the readout is larger.
			   Because timeouts and all that. */
			if (tlm->consumed > tlm->telemetry.Consumed) tlm->telemetry.Consumed = tlm->consumed;

			ActuatorTelemetrySet(&tlm->telemetry);
			tlm->push = 0;
			tlm->voltage = 0;
			tlm->consumed = 0;
		}
	}
#endif

	for (int n = 0; n < MAX_MIX_ACTUATORS; ++n) {
		PIOS_Servo_Set(n, command.Channel[n]);
	}

	PIOS_Servo_Update();
}

static void normalize_input_data(uint32_t this_systime,
		float (*desired_vect)[MIXERSETTINGS_MIXER1VECTOR_NUMELEM],
		bool *armed, bool *spin_while_armed, bool *stabilize_now)
{
	static float manual_throt = -1;
	float throttle_val = -1;
	ActuatorDesiredData desired;

	static FlightStatusData flightStatus;

	ActuatorDesiredGet(&desired);

	if (flight_status_updated) {
		FlightStatusGet(&flightStatus);
		flight_status_updated = false;
	}

	if (manual_control_cmd_updated) {
		// just pull out the throttle_val... and accessory0-2 and
		// fill direct into the vect
		ManualControlCommandThrottleGet(&manual_throt);
		manual_control_cmd_updated = false;
		ManualControlCommandAccessoryGet(
			&(*desired_vect)[MIXERSETTINGS_MIXER1VECTOR_ACCESSORY0]);
	}

	*armed = flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED;
	*spin_while_armed = actuatorSettings.MotorsSpinWhileArmed == ACTUATORSETTINGS_MOTORSSPINWHILEARMED_TRUE;

	if (airframe_type == SYSTEMSETTINGS_AIRFRAMETYPE_HELICP) {
		// Helis set throttle from manual control's throttle value,
		// unless in failsafe.
		if (flightStatus.FlightMode != FLIGHTSTATUS_FLIGHTMODE_FAILSAFE) {
			throttle_val = manual_throt;
		}
	} else {
		throttle_val = desired.Thrust;
	}

	if (!*armed) {
		throttle_val = -1;
	}

	*stabilize_now = throttle_val > 0.0f;

	float val1 = throt_curve(throttle_val, curve1,
			MIXERSETTINGS_THROTTLECURVE1_NUMELEM);

	//The source for the secondary curve is selectable
	float val2 = collective_curve(
			get_curve2_source(&desired, airframe_type, curve2_src),
			curve2, MIXERSETTINGS_THROTTLECURVE2_NUMELEM);

	fill_desired_vector(&desired, val1, val2, desired_vect);
}

/**
 * @brief Main Actuator module task
 *
 * Universal matrix based mixer for VTOL, helis and fixed wing.
 * Converts desired roll,pitch,yaw and throttle to servo/ESC outputs.
 *
 * Because of how the Throttle ranges from 0 to 1, the motors should too!
 *
 * Note this code depends on the UAVObjects for the mixers being all being the same
 * and in sequence. If you change the object definition, make sure you check the code!
 *
 * @return -1 if error, 0 if success
 */
static void actuator_task(void* parameters)
{
	// Connect update callbacks
	FlightStatusConnectCallbackCtx(UAVObjCbSetFlag, &flight_status_updated);
	ManualControlCommandConnectCallbackCtx(UAVObjCbSetFlag, &manual_control_cmd_updated);

	// Ensure the initial state of actuators is safe.
	actuator_settings_updated = false;
	ActuatorSettingsGet(&actuatorSettings);

	PIOS_Servo_SetMode(actuatorSettings.TimerUpdateFreq,
			ACTUATORSETTINGS_TIMERUPDATEFREQ_NUMELEM,
			actuatorSettings.ChannelMax,
			actuatorSettings.ChannelMin);
	set_failsafe();

	/* This is out here because not everything may change each time */
	uint32_t last_systime = PIOS_Thread_Systime();
	float desired_vect[MIXERSETTINGS_MIXER1VECTOR_NUMELEM] = { 0 };
	float dT = 0.0f;
	
	float maxpoweradd_bucket = 0.0f;

	// Main task loop
	while (1) {
		/* If settings objects have changed, update our internal
		 * state appropriately.
		 */
		if (actuator_settings_updated) {
			actuator_settings_updated = false;
			ActuatorSettingsGet(&actuatorSettings);

			PIOS_Servo_SetMode(actuatorSettings.TimerUpdateFreq,
					ACTUATORSETTINGS_TIMERUPDATEFREQ_NUMELEM,
					actuatorSettings.ChannelMax,
					actuatorSettings.ChannelMin);
		}

		if (mixer_settings_updated) {
			mixer_settings_updated = false;
			SystemSettingsAirframeTypeGet(&airframe_type);

			compute_mixer();
			// XXX compute_inverse_mixer();

			MixerSettingsThrottleCurve1Get(curve1);
			MixerSettingsThrottleCurve2Get(curve2);
			MixerSettingsCurve2SourceGet(&curve2_src);
		}

		PIOS_WDG_UpdateFlag(PIOS_WDG_ACTUATOR);

		UAVObjEvent ev;

		// Wait until the ActuatorDesired object is updated
		if (!PIOS_Queue_Receive(queue, &ev, FAILSAFE_TIMEOUT_MS)) {
			// If we hit a timeout, set the actuator failsafe and
			// try again.
			set_failsafe();
			continue;
		}

		uint32_t this_systime = PIOS_Thread_Systime();

		/* Check how long since last update; this is stored into the
		 * UAVO to allow analysis of actuation jitter.
		 */
		if (this_systime > last_systime) {
			dT = (this_systime - last_systime) / 1000.0f;
			/* (Otherwise, the timer has wrapped [rare] and we should
			 * just reuse dT)
			 */
		}

		last_systime = this_systime;

		if (actuator_interlock != ACTUATOR_INTERLOCK_OK) {
			/* Chosen because: 50Hz does 4-6 updates in 100ms */
			uint32_t exp_time = this_systime + 100;

			while (actuator_interlock != ACTUATOR_INTERLOCK_OK) {
				/* Simple state machine.  If someone has asked us to
				 * stop, set actuator failsafe for a short while.
				 * Then, set the flag to STOPPED.
				 *
				 * Setting to STOPPED isn't atomic, so we rely on
				 * anyone who has stopped us to waitfor STOPPED
				 * before putting us back to OK.
				 */
				if (actuator_interlock == ACTUATOR_INTERLOCK_STOPREQUEST) {
					set_failsafe();

					this_systime = PIOS_Thread_Systime();

					if ((exp_time - this_systime) > 100) {
						actuator_interlock = ACTUATOR_INTERLOCK_STOPPED;
					}
				}

				PIOS_Thread_Sleep(3);
				PIOS_WDG_UpdateFlag(PIOS_WDG_ACTUATOR);
			}

			PIOS_Servo_SetMode(actuatorSettings.TimerUpdateFreq,
					ACTUATORSETTINGS_TIMERUPDATEFREQ_NUMELEM,
					actuatorSettings.ChannelMax,
					actuatorSettings.ChannelMin);
			continue;
		}


		float motor_vect[MAX_MIX_ACTUATORS];

		bool armed, spin_while_armed, stabilize_now;

		/* Receive manual control and desired UAV objects.  Perform
		 * arming / hangtime checks; form a vector with desired
		 * axis actions.
		 */
		normalize_input_data(this_systime, &desired_vect, &armed,
				&spin_while_armed, &stabilize_now);

		/* Multiply the actuators x desired matrix by the
		 * desired x 1 column vector. */
		matrix_mul_check(motor_mixer, desired_vect, motor_vect,
				MAX_MIX_ACTUATORS,
				MIXERSETTINGS_MIXER1VECTOR_NUMELEM,
				1);

		/* Perform clipping adjustments on the outputs, along with
		 * state-related corrections (spin while armed, disarmed, etc).
		 *
		 * Program the actual values to the timer subsystem.
		 */
		post_process_scale_and_commit(motor_vect, desired_vect,
				dT, armed, spin_while_armed, stabilize_now,
				&maxpoweradd_bucket);

		/* If we got this far, everything is OK. */
		AlarmsClear(SYSTEMALARMS_ALARM_ACTUATOR);
	}
}

/**
 * Interpolate a throttle curve
 *
 * throttle curve assumes input is [0,1]
 * this means that the throttle channel neutral value is nearly the same as its min value
 * this is convenient for throttle, since the neutral value is used as a failsafe and would thus shut off the motor
 *
 * @param input the input value, in [0,1]
 * @param curve the array of points in the curve
 * @param num_points the number of points in the curve
 * @return the output value, in [0,1]
 */
static float throt_curve(float const input, float const * curve, uint8_t num_points)
{
	return linear_interpolate(input, curve, num_points, 0.0f, 1.0f);
}

/**
 * Interpolate a collective curve
 *
 * we need to accept input in [-1,1] so that the neutral point may be set arbitrarily within the typical channel input range, which is [-1,1]
 *
 * @param input The input value, in [-1,1]
 * @param curve Array of points in the curve
 * @param num_points Number of points in the curve
 * @return the output value, in [-1,1]
 */
static float collective_curve(float const input, float const * curve, uint8_t num_points)
{
	return linear_interpolate(input, curve, num_points, -1.0f, 1.0f);
}

/**
 * Convert channel from -1/+1 to servo pulse duration in microseconds
 */
static float scale_channel(float value, int idx)
{
	float max = actuatorSettings.ChannelMax[idx];
	float min = actuatorSettings.ChannelMin[idx];
	float neutral = actuatorSettings.ChannelNeutral[idx];

	float valueScaled;
	// Scale
	if (value >= 0.0f) {
		valueScaled = value*(max-neutral) + neutral;
	} else {
		valueScaled = value*(neutral-min) + neutral;
	}

	if (max>min) {
		if (valueScaled > max) valueScaled = max;
		if (valueScaled < min) valueScaled = min;
	} else {
		if (valueScaled < max) valueScaled = max;
		if (valueScaled > min) valueScaled = min;
	}

	return valueScaled;
}

static float channel_failsafe_value(int idx)
{
	switch (types_mixer[idx]) {
	case MIXERSETTINGS_MIXER1TYPE_MOTOR:
		return actuatorSettings.ChannelMin[idx];
	case MIXERSETTINGS_MIXER1TYPE_SERVO:
		return actuatorSettings.ChannelNeutral[idx];
	case MIXERSETTINGS_MIXER1TYPE_DISABLED:
		return -1;
	default:
		/* Other channel types-- camera.  Center them. */
		return 0;
	}

}

/**
 * Set actuator output to the neutral values (failsafe)
 */
static void set_failsafe()
{
	float Channel[ACTUATORCOMMAND_CHANNEL_NUMELEM] = {0};

	// Set alarm
	AlarmsSet(SYSTEMALARMS_ALARM_ACTUATOR, SYSTEMALARMS_ALARM_CRITICAL);

	// Update servo outputs
	for (int n = 0; n < MAX_MIX_ACTUATORS; ++n) {
		float fs_val = channel_failsafe_value(n);

		Channel[n] = fs_val;

		PIOS_Servo_Set(n, fs_val);
	}

	PIOS_Servo_Update();

	// Update output object's parts that we changed
	ActuatorCommandChannelSet(Channel);
}

/**
 * @}
 * @}
 */
