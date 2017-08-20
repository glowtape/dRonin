/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_SENSORS Generic sensor interface functions
 * @brief Generic interface for sensors
 * @{
 *
 * @file       pios_sensors.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2013
 * @brief      Generic interface for sensors
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************/
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
 */

#include "pios_sensors.h"
#include "pios_semaphore.h"
#include "pios_delay.h"
#include <stddef.h>

struct pios_sensors_info {
	uint16_t flags;							/* Basic info about the device configuration. */
	uint8_t cues;							/* Whether the sensor has been cued. And what buses. */
	uint32_t last_update;					/* Last time the sensor was read. */
	float range;							/* Sensor range. */
	uint32_t samplerate;					/* Expected sample rate. */
	void *device;							/* Pointer to the device handle/struct. */
	pios_sensors_callback callback;			/* Sensor read-out callback. */
	union {
		pios_sensors_cue_call cue_call;		/* Sensor cueing function. */
		void *data;							/* Buffer for handling waiting on legacy queues. */
	} v;
	uint32_t pause;							/* Time to pause between reads for scheduled/polled sensors. */
	bool updated;							/* Semaphore for cheapskates. Signals updates from the driver. */
	union {
		struct pios_semaphore *semaphore;	/* ISR semaphore. Alternatively stores a queue handle. */
		struct pios_queue *queue;			/* Pointer to legacy queue. */
	} u;
	union {
		struct pios_sensors_info *next;		/* Point to next sibling. Prep for future use. */
		enum pios_sensor_type sensor_type;	/* Cache sensor type for legacy queue stuff. */
	} w;
};

pios_sensor_t PIOS_Sensors_Allocate(enum pios_sensor_type sensor_type);
void PIOS_Sensors_RegisterQueue(enum pios_sensor_type sensor_type, struct pios_queue *queue);

/* Deprecated function, for backward compatibility. */
int32_t PIOS_SENSORS_Init()
{
	PIOS_Sensors_Initialize();
	return 0;
}

/* Deprecated function, for backward compatibility. */
int32_t PIOS_SENSORS_Register(enum pios_sensor_type type, struct pios_queue *queue)
{
	if (PIOS_Sensors_Available(type))
		return -1;

	PIOS_Sensors_RegisterQueue(type, queue);
	return 0;
}

/* Deprecated function, for backward compatibility. */
bool PIOS_SENSORS_IsRegistered(enum pios_sensor_type type)
{
	return PIOS_Sensors_Available(type);
}

/* Deprecated function, for backward compatibility. */
void PIOS_SENSORS_SetMaxGyro(int32_t rate)
{
	pios_sensor_t s = PIOS_Sensors_GetSensor(PIOS_SENSOR_GYRO);

	if (!s) {
		/*  Plenty of times in old drivers, values get set before the sensor is registered.
			Register sensor early and mark as such, to tell RegisterQueue. */
		s = PIOS_Sensors_Allocate(PIOS_SENSOR_GYRO);
		s->flags |= PIOS_SENSORS_FLAG_EARLY_REG | PIOS_SENSORS_FLAG_QUEUE;
	}

	PIOS_Sensors_SetRange(s, rate);
}

/* Deprecated function, for backward compatibility. */
int32_t PIOS_SENSORS_GetMaxGyro()
{
	pios_sensor_t s = PIOS_Sensors_GetSensor(PIOS_SENSOR_GYRO);

	return s ? (int32_t)PIOS_Sensors_GetRange(s) : 0;
}

/* Deprecated function, for backward compatibility. */
void PIOS_SENSORS_SetSampleRate(enum pios_sensor_type type, uint32_t sample_rate)
{
	if (type >= PIOS_SENSOR_LAST)
		return;

	pios_sensor_t s = PIOS_Sensors_GetSensor(type);
	if (!s) {
		/*  Plenty of times in old drivers, values get set before the sensor is registered.
			Register sensor early and mark as such, to tell RegisterQueue. */
		s = PIOS_Sensors_Allocate(type);
		s->flags |= PIOS_SENSORS_FLAG_EARLY_REG | PIOS_SENSORS_FLAG_QUEUE;
	}

	PIOS_Sensors_SetUpdateRate(s, sample_rate);
}

/* Deprecated function, for backward compatibility. */
uint32_t PIOS_SENSORS_GetSampleRate(enum pios_sensor_type type)
{
	if (type >= PIOS_SENSOR_LAST)
		return 0;

	return PIOS_Sensors_GetUpdateRate(PIOS_Sensors_GetSensor(type));
}

/* Deprecated function, for backward compatibility. */
void PIOS_SENSORS_SetMissing(enum pios_sensor_type type)
{
	PIOS_Assert(type < PIOS_SENSOR_LAST);
#ifdef PIOS_TOLERATE_MISSING_SENSORS
	PIOS_Sensors_SetMissing(type);
#endif
}

/* Deprecated function, for backward compatibility. */
bool PIOS_SENSORS_GetMissing(enum pios_sensor_type type)
{
	PIOS_Assert(type < PIOS_SENSOR_LAST);

#ifdef PIOS_TOLERATE_MISSING_SENSORS
	return PIOS_Sensors_IsMissing(type);
#else
	return false;
#endif
}

/* 250ms timeout on cued sensors. */
#define CUE_TIMEOUT_uS				250000

uint8_t sensors_cues;						/* Stores what buses are used in cues. */
pios_sensor_t info[PIOS_SENSOR_LAST];		/* Sensor data per type. */

/*
 * @brief Initializes the sensor subsystem.
 */
void PIOS_Sensors_Initialize()
{
	for (int i = 0; i < PIOS_SENSOR_LAST; i++) {
		info[i] = NULL;
	}
}

/*
 * @brief Allocates the struct and adds it to the linked list of a given sensor type.
 */
pios_sensor_t PIOS_Sensors_Allocate(enum pios_sensor_type sensor_type)
{
	PIOS_Assert(sensor_type < PIOS_SENSOR_LAST);

	if (info[sensor_type]) {
		// Only multiple gyros and accels supported for now.
		PIOS_Assert(0);
	}

	pios_sensor_t s = PIOS_malloc_no_dma(sizeof(*s));
	PIOS_Assert(s);
	memset(s, 0, sizeof(*s));

	if (!info[sensor_type]) {
		info[sensor_type] = s;
	} else {
		PIOS_Assert(0);
	}

	return s;
}

/*
 * @brief Registers a sensor and its settings and callback.
 *
 * @param[in] sensor_type		The type of sensor (PIOS_SENSOR_*)
 * @param[in] device			Pointer to the device handle/configuration.
 * @param[in] callback			Pointer to the sensor read-out callback.
 * @param[in] cue_call			Pointer to the sensor cueing call.
 * @param[in] flags				Sensor flags.
 *
 * @returns Sensor handle.
 */
pios_sensor_t PIOS_Sensors_Register(enum pios_sensor_type sensor_type, void *device, const pios_sensors_callback callback,
	const pios_sensors_cue_call cue_call, uint16_t flags)
{
	PIOS_Assert(sensor_type < PIOS_SENSOR_LAST);

	if (flags & PIOS_SENSORS_FLAG_QUEUE) {
		// Don't allow manual registration of queues.
		PIOS_Assert(0);
	}

	pios_sensor_t s = PIOS_Sensors_Allocate(sensor_type);

	s->flags = flags;
	s->callback = callback;
	s->v.cue_call = cue_call;
	s->device = device;

	if (s->flags & PIOS_SENSORS_FLAG_SEMAPHORE) {
		s->u.semaphore = PIOS_Semaphore_Create();
		PIOS_Assert(s->u.semaphore);
	}

	return s;
}

/*
 * @brief Sets the range of a sensor.
 *
 * @param[in] s					Sensor handle.
 * @param[in] range				Range of the sensor.
 */
void PIOS_Sensors_SetRange(pios_sensor_t s, float range)
{
	PIOS_Assert(s);
	s->range = range;
}

/*
 * @brief Sets the update/sample rate of a sensor.
 *
 * @param[in] s					Sensor handle.
 * @param[in] samplerate		Sample rate of the sensor.
 */
void PIOS_Sensors_SetUpdateRate(pios_sensor_t s, uint32_t samplerate)
{
	PIOS_Assert(s);
	s->samplerate = samplerate;
}

/*
 * @brief Signals the sensor subsystem that sensor data is available.
 *
 * @param[in] s					Sensor handle.
 */
void PIOS_Sensors_RaiseSignal(pios_sensor_t s)
{
	PIOS_Assert(s);

	s->updated = true;

	if (s->u.semaphore) {
		PIOS_Semaphore_Give(s->u.semaphore);
	}
}

/*
 * @brief Signals the sensor subsystem from within an ISR, that sensor data is available.
 *
 * @param[in] s					Sensor handle.
 */
void PIOS_Sensors_RaiseSignal_FromISR(pios_sensor_t s)
{
	PIOS_Assert(s);

	s->updated = true;

	if (s->u.semaphore) {
		bool need_yield;
		PIOS_Semaphore_Give_FromISR(s->u.semaphore, &need_yield);
	}
}

/*
 * @brief Tells whether a sensor of a type of registered.
 *
 * @param[in] sensor_type		Type of sensor (PIOS_SENSOR_*)
 *
 * @returns True if there's a sensor, false if there's none.
 */
bool PIOS_Sensors_Available(enum pios_sensor_type sensor_type)
{
	PIOS_Assert(sensor_type < PIOS_SENSOR_LAST);
	return info[sensor_type] && (info[sensor_type]->callback || info[sensor_type]->u.queue);
}

/*
 * @brief Checks whether a scheduled sensor is due to run.
 */
inline bool PIOS_Sensors_Unpause(pios_sensor_t s)
{
	return PIOS_DELAY_GetuSSince(s->last_update) > s->pause;
}

/*
 * @brief Checks whether a cue timed out.
 */
inline bool PIOS_Sensors_Timeout(pios_sensor_t s)
{
	return PIOS_DELAY_GetuSSince(s->last_update) > CUE_TIMEOUT_uS;
}

/*
 * @brief Cues a sensor.
 */
int PIOS_Sensors_Cue(pios_sensor_t s)
{
	PIOS_Assert(s);
	PIOS_Assert(s->v.cue_call);

	if (sensors_cues & (s->flags & PIOS_SENSORS_CUE_MASK)) {
		/* There's already something cued with the desired buses. */
		return -1;
	}

	s->cues = (uint8_t)(s->flags & PIOS_SENSORS_CUE_MASK);
	/* Register globally what buses are used in the cue. */
	sensors_cues |= s->cues;

	/* Move timestamp forward to deal with the timeout. */
	s->last_update = PIOS_DELAY_GetuS();

	/* Tell the sensor to do its thing. */
	s->v.cue_call(s->device);

	return 0;
}

/*
 * @brief Retrieves (potential) data for a sensor of a type.
 *
 * @param[in] s					Sensor handle.
 * @param[out] output			The memory pointer to write to.
 */
int PIOS_Sensors_GetData(pios_sensor_t s, void *output)
{
	PIOS_Assert(output);
	PIOS_Assert(s);

	if (!s || (s->flags & PIOS_SENSORS_FLAG_MISSING)) {
		/* Yea well... */
		return PIOS_SENSORS_DATA_ERROR;
	}

	/* Sensor is a wrapped queue. */
	if (s->flags & PIOS_SENSORS_FLAG_QUEUE) {
		if (s->v.data) {
			/* We're dumping data in _wait_on, pass it over. We're expecting this function
			   to be gated on _wait_on, so we're not checking whether the data changed. */
			memcpy(output, s->v.data, PIOS_Queue_GetItemSize(s->u.queue));
			return PIOS_SENSORS_DATA_AVAILABLE;
		} else {
			/* We're directly polling the queue. */
			PIOS_Assert(s->u.queue);
			bool result = PIOS_Queue_Receive(s->u.queue, output, 0);
			if (result) {
				s->last_update = PIOS_DELAY_GetuS();
			}
			return result ? PIOS_SENSORS_DATA_AVAILABLE : PIOS_SENSORS_DATA_NONE;
		}
	} else {
		/* If this ain't a wrapped queue, we kind of want a callback function. */
		PIOS_Assert(s->callback);
	}

	/* Sensor can be cued, but no cues are running */
	if ((s->flags & PIOS_SENSORS_CUE_MASK) && !s->cues) {
		if ((s->flags & PIOS_SENSORS_FLAG_SCHEDULED) && !PIOS_Sensors_Unpause(s)) {
			/* If sensor is on a schedule, don't cue things */
			return PIOS_SENSORS_DATA_NONE;
		}
		/* Sensor needs cueing. Do so. */
		return PIOS_Sensors_Cue(s) ? PIOS_SENSORS_DATA_BUSY : PIOS_SENSORS_DATA_WAITING;
	}

	/* Sensor has been cued, but not updated. */
	if (s->cues && !s->updated) {
		if (PIOS_Sensors_Timeout(s)) {
			/* Sensor cue timed out. That's not good, declare sensor missing.
			   Next round it'll fail at the top of this function. */
			s->flags |= PIOS_SENSORS_FLAG_MISSING;
			s->cues = 0;
			return PIOS_SENSORS_DATA_ERROR;
		}
		/* Sensor is still cued, return no data. */
		return PIOS_SENSORS_DATA_WAITING;
	}

	int data_state = PIOS_SENSORS_DATA_NONE;

	/* If sensor updated or is polled/scheduled, go do some stuff. Scheduled sensors should
	   be gated on _get_lru, so we don't check here whether they're actually up for duty. */
	if (s->updated || ((s->flags & PIOS_SENSORS_FLAG_SCHEDULED) && PIOS_Sensors_Unpause(s))) {
		data_state = s->callback(s->device, output);
		s->updated = false;
		s->last_update = PIOS_DELAY_GetuS();
		/* Consider the cue finished after a call to this function. */
		sensors_cues &= ~((uint8_t)(s->flags & PIOS_SENSORS_CUE_MASK));
		s->cues = 0;
	}

	return data_state;
}

/*
 * @brief Returns the least recently used sensor type.
 *
 * @returns Sensor type (PIOS_SENSOR_*) that's up next, otherwise 0xFF.
 */
uint8_t PIOS_Sensors_GetLRU()
{
	uint8_t sensor = 0xFF;
	uint32_t delta_t = 0;

	for (int i = 0; i < PIOS_SENSOR_LAST; i++) {
		if ((i == PIOS_SENSOR_GYRO) || (i == PIOS_SENSOR_ACCEL)) {
			/* Gyros and accels considered primary sensors.
			   They should be polled upstreams regardless. */
			continue;
		}

		/* Secondary sensors don't do multiples, so pick first only. */
		pios_sensor_t s = info[i];

		if (!s || s->cues || (s->flags & PIOS_SENSORS_FLAG_MISSING)) {
			/* No sensor. Or sensor has been cued. */
			continue;
		}

		if (s->flags & PIOS_SENSORS_FLAG_SCHEDULED) {
			/* Sensor runs on a schedule, does it need to run? */
			if (!PIOS_Sensors_Unpause(s))
				continue;
		}

		uint32_t x = PIOS_DELAY_GetuSSince(s->last_update);
		if (x > delta_t) {
			delta_t = x;
			sensor = i;
		}
	}

	return sensor;
}

/*
 * @brief Returns the range of a sensor.
 *
 * @param[in] s					Sensor handle.
 *
 * @returns Range (from 0 to MAX).
 */
float PIOS_Sensors_GetRange(pios_sensor_t s)
{
	PIOS_Assert(s);
	return s->range;
}

/*
 * @brief Returns the update/sample rate of a sensor.
 *
 * @param[in] s					Sensor handle.
 *
 * @returns Sample rate in Hz.
 */
uint32_t PIOS_Sensors_GetUpdateRate(pios_sensor_t s)
{
	PIOS_Assert(s);
	return s->samplerate;
}

/*
 * @brief Waits for a sensor to trigger.
 *
 * @param[in] s					Sensor handle.
 * @param[in] timeout			Timeout for the sensor in milliseconds.
 *
 * @returns True if the sensor triggered, false if it timed out.
 */
bool PIOS_Sensors_WaitOn(pios_sensor_t s, int timeout)
{
	PIOS_Assert(s);

	if (s->flags & PIOS_SENSORS_FLAG_MISSING) {
		/* Fail directly, if the sensor is declared missing. */
		return false;
	} else if (s->flags & PIOS_SENSORS_FLAG_QUEUE) {
		/* Check if the sensor's fully registered. */
		if (s->flags & PIOS_SENSORS_FLAG_EARLY_REG) {
			/* We shouldn't even get here, if things went well during board init. */
			s->flags |= PIOS_SENSORS_FLAG_MISSING;
			return false;
		}
		/* If we're waiting on a wrapped queued sensor, dump
		   the data in a holding buffer. */
		if (!s->v.data) {
			s->v.data = PIOS_malloc_no_dma(PIOS_Queue_GetItemSize(s->u.queue));
			PIOS_Assert(s->v.data);
		}
		bool result = PIOS_Queue_Receive(s->u.queue, s->v.data, timeout);
		if (result) {
			s->last_update = PIOS_DELAY_GetuS();
		}
		return result;
	} else if (s->u.semaphore) {
		if (s->updated) {
			/* Sensor updated before we're taking the semaphore.  */
			PIOS_Semaphore_Take(s->u.semaphore, 0);
			return true;
		} else {
			return PIOS_Semaphore_Take(s->u.semaphore, timeout);
		}
	} else {
		return false;
	}
}

/*
 * @brief Schedules the updates of a sensor.
 *
 * @param[in] s					Sensor handle.
 * @param[in] microseconds		Desired time between updates in microseconds.
 */
void PIOS_Sensors_SetSchedule(pios_sensor_t s, uint32_t microseconds)
{
	PIOS_Assert(s && s->callback);
	s->last_update = PIOS_DELAY_GetuS();
	s->pause = microseconds;
}

/*
 * @brief Marks a sensor as missing.
 *
 * @param[in] sensor_type		Type of sensor (PIOS_SENSOR_*)
 */
void PIOS_Sensors_SetMissing(enum pios_sensor_type sensor_type)
{
	PIOS_Assert(sensor_type < PIOS_SENSOR_LAST);
	pios_sensor_t s = info[sensor_type];

	if (s) s->flags |= PIOS_SENSORS_FLAG_MISSING;
}

/*
 * @brief Checks whether a sensor is marked missing or not.
 *
 * @param[in] sensor_type		Type of sensor (PIOS_SENSOR_*)
 *
 * @returns True if the sensor is gone, false if not.
 */
bool PIOS_Sensors_IsMissing(enum pios_sensor_type sensor_type)
{
	PIOS_Assert(sensor_type < PIOS_SENSOR_LAST);
	pios_sensor_t s = info[sensor_type];
	return (!s || (s->flags & PIOS_SENSORS_FLAG_MISSING));
}

/*
 * @brief Wraps an old queue into the new system. For internal use only.
 */
void PIOS_Sensors_RegisterQueue(enum pios_sensor_type sensor_type, struct pios_queue *queue)
{
	PIOS_Assert(sensor_type < PIOS_SENSOR_LAST);
	pios_sensor_t s = info[sensor_type];

	if (s && !(s->flags & PIOS_SENSORS_FLAG_EARLY_REG))
		return;

	/* No sensor of type, add the queue. */
	if (!s) {
		s = PIOS_Sensors_Allocate(sensor_type);
	}
	else
	{
		s->flags &= ~PIOS_SENSORS_FLAG_EARLY_REG;
	}

	s->flags |= PIOS_SENSORS_FLAG_QUEUE;
	s->u.queue = queue;
	s->w.sensor_type = sensor_type;
}

/*
 * @brief Returns the sensor handle of a specific type.
 *
 * @param[in] sensor_type		Type of sensor (PIOS_SENSOR_*)
 *
 * @returns Sensor handle.
 */
pios_sensor_t PIOS_Sensors_GetSensor(enum pios_sensor_type sensor_type)
{
	PIOS_Assert(sensor_type < PIOS_SENSOR_LAST);
	return info[sensor_type];
}
