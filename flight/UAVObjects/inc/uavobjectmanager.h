/******************************************************************************
 * @addtogroup FlightCore Core components
 * @{
 * @addtogroup UAVObjectHandling UAVObject handling code
 * @{
 *
 * @file       uavobjectmanager.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2014
 * @author     dRonin, http://dronin.org Copyright (C) 2015-2016
 * @brief      Object manager library. This library holds a collection of all objects.
 *             It can be used by all modules/libraries to find an object reference.
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

#ifndef UAVOBJECTMANAGER_H
#define UAVOBJECTMANAGER_H

#include "pios_queue.h"

#define UAVOBJ_ALL_INSTANCES 0xFFFF
#define UAVOBJ_MAX_INSTANCES 1000

/*
 * Shifts and masks used to read/write metadata flags.
 */
#define UAVOBJ_ACCESS_SHIFT 0
#define UAVOBJ_GCS_ACCESS_SHIFT 1
#define UAVOBJ_TELEMETRY_ACKED_SHIFT 2
#define UAVOBJ_GCS_TELEMETRY_ACKED_SHIFT 3
#define UAVOBJ_TELEMETRY_UPDATE_MODE_SHIFT 4
#define UAVOBJ_GCS_TELEMETRY_UPDATE_MODE_SHIFT 6
#define UAVOBJ_UPDATE_MODE_MASK 0x3

typedef struct UAVOBase * UAVObjHandle;

/**
 * Object update mode, used by multiple modules (e.g. telemetry and logger)
 */
typedef enum {
	UPDATEMODE_MANUAL = 0, /** Manually update object, by calling the updated() function */
	UPDATEMODE_PERIODIC = 1, /** Automatically update object at periodic intervals */
	UPDATEMODE_ONCHANGE = 2, /** Only update object when its data changes */
	UPDATEMODE_THROTTLED = 3 /** Object is updated on change, but not more often than the interval time */
} UAVObjUpdateMode;

/**
 * Object metadata, each object has a meta object that holds its metadata. The metadata define
 * properties for each object and can be used by multiple modules (e.g. telemetry and logger)
 *
 * The object metadata flags are packed into a single 16 bit integer.
 * The bits in the flag field are defined as:
 *
 *   Bit(s)  Name                     Meaning
 *   ------  ----                     -------
 *      0    access                   Defines the access level for the local transactions (readonly=1 and readwrite=0)
 *      1    gcsAccess                Defines the access level for the local GCS transactions (readonly=1 and readwrite=0), not used in the flight s/w
 *      2    telemetryAcked           Defines if an ack is required for the transactions of this object (1:acked, 0:not acked)
 *      3    gcsTelemetryAcked        Defines if an ack is required for the transactions of this object (1:acked, 0:not acked)
 *    4-5    telemetryUpdateMode      Update mode used by the telemetry module (UAVObjUpdateMode)
 *    6-7    gcsTelemetryUpdateMode   Update mode used by the GCS (UAVObjUpdateMode)
 */
typedef struct {
	uint8_t flags; /** Defines flags for update and logging modes and whether an update should be ACK'd (bits defined above) */
	uint16_t telemetryUpdatePeriod; /** Update period used by the telemetry module (only if telemetry mode is PERIODIC) */
	uint16_t gcsTelemetryUpdatePeriod; /** Update period used by the GCS (only if telemetry mode is PERIODIC) */
	uint16_t loggingUpdatePeriod; /** Update period used by the logging module (only if logging mode is PERIODIC) */
} __attribute__((packed)) UAVObjMetadata;

/**
 * Event types generated by the objects.
 */
typedef enum {
	EV_NONE = 0x00, /** No event */
	EV_UNPACKED = 0x01, /** Object data updated by unpacking */
	EV_UPDATED = 0x02, /** Object data updated by changing the data structure */
	EV_UPDATED_MANUAL = 0x04, /** Object update event manually generated */
	EV_UPDATED_PERIODIC = 0x08, /** Object update from periodic event */
} UAVObjEventType;

/**
 * Helper macros for event masks
 */
#define EV_MASK_ALL 0
#define EV_MASK_ALL_UPDATES (EV_UNPACKED | EV_UPDATED | EV_UPDATED_MANUAL | EV_UPDATED_PERIODIC)

/**
 * Access types
 */
typedef enum {
	ACCESS_READWRITE = 0,
	ACCESS_READONLY = 1
} UAVObjAccessType;

/**
 * Event message, this structure is sent in the event queue each time an event is generated
 */
typedef struct {
	UAVObjHandle obj;
	UAVObjEventType event;
	struct ObjectEventEntryThrottled *throttle;

	uint16_t instId;
} __attribute__((packed)) UAVObjEvent;

/**
 * Event callback, this function is called when an event is invoked. The function
 * will be executed in the event task. The ev parameter should be copied if needed
 * after the function returns.
 */
typedef void (*UAVObjEventCallback)(UAVObjEvent* ev, void* cb_ctx,
	void *uavo_data, int uavo_len);

/**
 * Callback used to initialize the object fields to their default values.
 */
typedef void (*UAVObjInitializeCallback)(UAVObjHandle obj_handle, uint16_t instId);

/**
 * Event manager statistics
 */
typedef struct {
	uint32_t eventQueueErrors;
	uint32_t eventCallbackErrors;
	uint32_t lastCallbackErrorID;
	uint32_t lastQueueErrorID;
} UAVObjStats;

typedef void (*new_uavo_instance_cb_t)(uint32_t,uint32_t);
void UAVObjRegisterNewInstanceCB(new_uavo_instance_cb_t callback);

int32_t UAVObjInitialize();
void UAVObjGetStats(UAVObjStats* statsOut);
void UAVObjClearStats();
UAVObjHandle UAVObjRegister(uint32_t id,
		int32_t isSingleInstance, int32_t isSettings, uint32_t numBytes, UAVObjInitializeCallback initCb);
UAVObjHandle UAVObjGetByID(uint32_t id);
uint32_t UAVObjGetID(UAVObjHandle obj);
uint32_t UAVObjGetNumBytes(UAVObjHandle obj);
uint16_t UAVObjGetNumInstances(UAVObjHandle obj);
UAVObjHandle UAVObjGetLinkedObj(UAVObjHandle obj);
uint16_t UAVObjCreateInstance(UAVObjHandle obj_handle, UAVObjInitializeCallback initCb);
bool UAVObjIsSingleInstance(UAVObjHandle obj);
bool UAVObjIsMetaobject(UAVObjHandle obj);
bool UAVObjIsSettings(UAVObjHandle obj);
int32_t UAVObjUnpack(UAVObjHandle obj_handle, uint16_t instId, const uint8_t* dataIn);
int32_t UAVObjPack(UAVObjHandle obj_handle, uint16_t instId, uint8_t* dataOut);
int32_t UAVObjSave(UAVObjHandle obj_handle, uint16_t instId);
int32_t UAVObjLoad(UAVObjHandle obj_handle, uint16_t instId);
int32_t UAVObjDeleteById(uint32_t obj_id, uint16_t inst_id);
#if defined(PIOS_INCLUDE_SDCARD)
int32_t UAVObjSaveToFile(UAVObjHandle obj_handle, uint16_t instId, FILEINFO* file);
UAVObjHandle UAVObjLoadFromFile(FILEINFO* file);
#endif
int32_t UAVObjSaveSettings();
int32_t UAVObjLoadSettings();
int32_t UAVObjDeleteSettings();
int32_t UAVObjSaveMetaobjects();
int32_t UAVObjLoadMetaobjects();
int32_t UAVObjDeleteMetaobjects();
int32_t UAVObjSetData(UAVObjHandle obj_handle, const void* dataIn);
int32_t UAVObjSetDataField(UAVObjHandle obj_handle, const void* dataIn, uint32_t offset, uint32_t size);
int32_t UAVObjGetData(UAVObjHandle obj_handle, void* dataOut);
int32_t UAVObjGetDataField(UAVObjHandle obj_handle, void* dataOut, uint32_t offset, uint32_t size);
int32_t UAVObjSetInstanceData(UAVObjHandle obj_handle, uint16_t instId, const void* dataIn);
int32_t UAVObjSetInstanceDataField(UAVObjHandle obj_handle, uint16_t instId, const void* dataIn, uint32_t offset, uint32_t size);
int32_t UAVObjGetInstanceData(UAVObjHandle obj_handle, uint16_t instId, void* dataOut);
int32_t UAVObjGetInstanceDataField(UAVObjHandle obj_handle, uint16_t instId, void* dataOut, uint32_t offset, uint32_t size);
int32_t UAVObjSetMetadata(UAVObjHandle obj_handle, const UAVObjMetadata* dataIn);
int32_t UAVObjGetMetadata(UAVObjHandle obj_handle, UAVObjMetadata* dataOut);
uint8_t UAVObjGetMetadataAccess(const UAVObjMetadata* dataOut);
UAVObjAccessType UAVObjGetAccess(const UAVObjMetadata* dataOut);
void UAVObjSetAccess(UAVObjMetadata* dataOut, UAVObjAccessType mode);
UAVObjAccessType UAVObjGetGcsAccess(const UAVObjMetadata* dataOut);
void UAVObjSetGcsAccess(UAVObjMetadata* dataOut, UAVObjAccessType mode);
uint8_t UAVObjGetTelemetryAcked(const UAVObjMetadata* dataOut);
void UAVObjSetTelemetryAcked( UAVObjMetadata* dataOut, uint8_t val);
uint8_t UAVObjGetGcsTelemetryAcked(const UAVObjMetadata* dataOut);
void UAVObjSetGcsTelemetryAcked(UAVObjMetadata* dataOut, uint8_t val);
UAVObjUpdateMode UAVObjGetTelemetryUpdateMode(const UAVObjMetadata* dataOut);
void UAVObjSetTelemetryUpdateMode(UAVObjMetadata* dataOut, UAVObjUpdateMode val);
UAVObjUpdateMode UAVObjGetGcsTelemetryUpdateMode(const UAVObjMetadata* dataOut);
void UAVObjSetTelemetryGcsUpdateMode(UAVObjMetadata* dataOut, UAVObjUpdateMode val);
int8_t UAVObjReadOnly(UAVObjHandle obj);
int32_t UAVObjConnectQueue(UAVObjHandle obj_handle, struct pios_queue *queue, uint8_t eventMask);
int32_t UAVObjDisconnectQueue(UAVObjHandle obj_handle, struct pios_queue *queue);
int32_t UAVObjConnectQueueThrottled(UAVObjHandle obj_handle, struct pios_queue *queue, uint8_t eventMask, uint16_t interval);
int32_t UAVObjConnectCallback(UAVObjHandle obj_handle, UAVObjEventCallback cb, void *cbCtx, uint8_t eventMask);
int32_t UAVObjConnectCallbackThrottled(UAVObjHandle obj_handle, UAVObjEventCallback cb, void *cbCtx, uint8_t eventMask, uint16_t interval);
void UAVObjUnblockThrottle(struct ObjectEventEntryThrottled *throttled);
int32_t UAVObjDisconnectCallback(UAVObjHandle obj_handle, UAVObjEventCallback cb, void *cbCtx);
void UAVObjUpdated(UAVObjHandle obj);
void UAVObjInstanceUpdated(UAVObjHandle obj_handle, uint16_t instId);
void UAVObjIterate(void (*iterator)(UAVObjHandle obj));
int32_t getEventMask(UAVObjHandle obj_handle, struct pios_queue *queue);
uint8_t UAVObjCount();
uint32_t UAVObjIDByIndex(uint8_t index);
void UAVObjCbSetFlag(UAVObjEvent *objEv, void *ctx, void *obj, int len);
void UAVObjCbCopyData(UAVObjEvent *objEv, void *ctx, void *obj, int len);

#endif // UAVOBJECTMANAGER_H

/**
 * @}
 * @}
 */
