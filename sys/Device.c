
#include "Driver.h"
#include "Device.tmh"
#include <DmfModule.h>
#include <devpkey.h>


EVT_DMF_DEVICE_MODULES_ADD DmfDeviceModulesAdd;

#pragma code_seg("PAGED")
DMF_DEFAULT_DRIVERCLEANUP(dshidminiEvtDriverContextCleanup)

//
// Bootstrap device
// 
NTSTATUS
dshidminiEvtDeviceAdd(
	_In_    WDFDRIVER       Driver,
	_Inout_ PWDFDEVICE_INIT DeviceInit
)
{
	WDF_OBJECT_ATTRIBUTES			deviceAttributes;
	WDFDEVICE						device;
	NTSTATUS						status;
	PDMFDEVICE_INIT					dmfDeviceInit;
	DMF_EVENT_CALLBACKS				dmfCallbacks;
	WDF_PNPPOWER_EVENT_CALLBACKS	pnpPowerCallbacks;
	PDEVICE_CONTEXT					pDevCtx;
	WDFQUEUE						queue;
	WDF_IO_QUEUE_CONFIG				queueConfig;
	BOOLEAN ret;
	

	UNREFERENCED_PARAMETER(Driver);

	FuncEntry(TRACE_DEVICE);

	dmfDeviceInit = DMF_DmfDeviceInitAllocate(DeviceInit);

	WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpPowerCallbacks);

	//
	// Callbacks only relevant to Bluetooth
	// 
	if ((NT_SUCCESS(DsDevice_IsUsbDevice(DeviceInit, &ret)) && !ret))
	{
		pnpPowerCallbacks.EvtDeviceSelfManagedIoInit = DsHidMini_EvtWdfDeviceSelfManagedIoInit;
		pnpPowerCallbacks.EvtDeviceSelfManagedIoSuspend = DsHidMini_EvtWdfDeviceSelfManagedIoSuspend;
	}	
	
	pnpPowerCallbacks.EvtDevicePrepareHardware = DsHidMini_EvtDevicePrepareHardware;
	pnpPowerCallbacks.EvtDeviceD0Entry = DsHidMini_EvtDeviceD0Entry;
	pnpPowerCallbacks.EvtDeviceD0Exit = DsHidMini_EvtDeviceD0Exit;

	// All DMF drivers must call this function even if they do not support PnP Power callbacks.
	// (In this case, this driver does support a PnP Power callback.)
	//
	DMF_DmfDeviceInitHookPnpPowerEventCallbacks(dmfDeviceInit,
		&pnpPowerCallbacks);

	// All DMF drivers must call this function even if they do not support File Object callbacks.
	//
	DMF_DmfDeviceInitHookFileObjectConfig(dmfDeviceInit,
		NULL);

	// All DMF drivers must call this function even if they do not support Power Policy callbacks.
	//
	DMF_DmfDeviceInitHookPowerPolicyEventCallbacks(dmfDeviceInit,
		NULL);

	// This is a filter driver that loads on MSHIDUMDF driver.
	//
	WdfFdoInitSetFilter(DeviceInit);
	// DMF Client drivers that are filter drivers must also make this call.
	//
	DMF_DmfFdoSetFilter(dmfDeviceInit);

	WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&deviceAttributes, DEVICE_CONTEXT);

	status = WdfDeviceCreate(&DeviceInit, &deviceAttributes, &device);

	do
	{
		if (!NT_SUCCESS(status))
		{
			TraceError(
				TRACE_DEVICE,
				"WdfDeviceCreate failed with status %!STATUS!",
				status
			);
			break;
		}

		//
		// Read device properties
		// 	
		status = DsDevice_ReadProperties(device);
		if (!NT_SUCCESS(status))
		{
			break;
		}

		pDevCtx = DeviceGetContext(device);

		//
		// Initialize context
		// 
		status = DsDevice_InitContext(device);
		if (!NT_SUCCESS(status))
		{
			TraceError(
				TRACE_DEVICE,
				"DsDevice_InitContext failed with status %!STATUS!",
				status
			);
			break;
		}

		if (pDevCtx->ConnectionType == DsDeviceConnectionTypeUsb)
		{
			//
			// Provide and hook our own default queue to handle weird cases
			//

			WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&queueConfig, WdfIoQueueDispatchParallel);
			queueConfig.PowerManaged = WdfTrue;
			queueConfig.EvtIoDeviceControl = DSHM_EvtWdfIoQueueIoDeviceControl;
			DMF_DmfDeviceInitHookQueueConfig(dmfDeviceInit, &queueConfig);

			status = WdfIoQueueCreate(
				device,
				&queueConfig,
				WDF_NO_OBJECT_ATTRIBUTES,
				&queue
			);
			if (!NT_SUCCESS(status))
			{
				TraceError(
					TRACE_DEVICE,
					"WdfIoQueueCreate failed with status %!STATUS!",
					status
				);
				break;
			}
		}
		
		//
		// Expose interface for applications to find us
		// 

		status = WdfDeviceCreateDeviceInterface(
			device,
			&GUID_DEVINTERFACE_DSHIDMINI,
			NULL
		);
		if (!NT_SUCCESS(status))
		{
			TraceError(
				TRACE_DEVICE,
				"WdfDeviceCreateDeviceInterface failed with status %!STATUS!",
				status
			);
			break;
		}

		// Create the DMF Modules this Client driver will use.
		//
		dmfCallbacks.EvtDmfDeviceModulesAdd = DmfDeviceModulesAdd;
		DMF_DmfDeviceInitSetEventCallbacks(
			dmfDeviceInit,
			&dmfCallbacks
		);

		status = DMF_ModulesCreate(
			device,
			&dmfDeviceInit
		);
		if (!NT_SUCCESS(status))
		{
			break;
		}
	} while (FALSE);

	if (dmfDeviceInit != NULL)
	{
		DMF_DmfDeviceInitFree(&dmfDeviceInit);
	}

	FuncExit(TRACE_DEVICE, "status=%!STATUS!", status);

	return status;
}
#pragma code_seg()

//
// Read device properties available on device creation
// 
NTSTATUS DsDevice_ReadProperties(WDFDEVICE Device)
{
	NTSTATUS status;
	WCHAR enumeratorName[200];
	ULONG bufSize;
	WDF_DEVICE_PROPERTY_DATA devProp;
	DEVPROPTYPE propType;
	WDF_OBJECT_ATTRIBUTES attributes;
	ULONG requiredSize = 0;
	PDEVICE_CONTEXT pDevCtx = DeviceGetContext(Device);

	FuncEntry(TRACE_DEVICE);
	
	do
	{
		//
		// Query enumerator name to discover connection type
		// 
		status = WdfDeviceQueryProperty(
			Device,
			DevicePropertyEnumeratorName,
			ARRAYSIZE(enumeratorName),
			(PVOID)enumeratorName,
			&bufSize
		);
		if (!NT_SUCCESS(status))
		{
			TraceError(
				TRACE_DEVICE,
				"WdfDeviceQueryProperty failed with status %!STATUS!",
				status
			);
			break;
		}
		
		//
		// Early device type detection, using enumerator name
		// 
		if (_wcsicmp(L"USB", enumeratorName) == 0)
		{
			pDevCtx->ConnectionType = DsDeviceConnectionTypeUsb;
		}
		else
		{
			pDevCtx->ConnectionType = DsDeviceConnectionTypeBth;
		}

		WDF_DEVICE_PROPERTY_DATA_INIT(&devProp, &DEVPKEY_Device_InstanceId);
		WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
		attributes.ParentObject = Device;

		//
		// Query device instance string for configuration
		// TODO: deprecated
		// 
		status = WdfDeviceAllocAndQueryPropertyEx(
			Device,
			&devProp,
			NonPagedPoolNx,
			&attributes,
			&pDevCtx->InstanceId,
			&propType
		);
		if (!NT_SUCCESS(status))
		{
			TraceError(
				TRACE_DEVICE,
				"WdfDeviceAllocAndQueryPropertyEx failed with status %!STATUS!",
				status
			);
			break;
		}

		TraceVerbose(
			TRACE_DEVICE,
			"DEVPKEY_Device_InstanceId: %ws",
			WdfMemoryGetBuffer(pDevCtx->InstanceId, NULL)
		);
		
		//
		// Fetch Bluetooth-specific properties
		// 
		if (pDevCtx->ConnectionType == DsDeviceConnectionTypeBth)
		{
			WDFMEMORY deviceAddressMemory;
			WDF_DEVICE_PROPERTY_DATA_INIT(&devProp, &DEVPKEY_Bluetooth_DeviceAddress);

			//
			// Get device property (returns wide hex string)
			// 
			status = WdfDeviceAllocAndQueryPropertyEx(
				Device,
				&devProp,
				NonPagedPoolNx,
				WDF_NO_OBJECT_ATTRIBUTES,
				&deviceAddressMemory,
				&propType
			);
			if (!NT_SUCCESS(status))
			{
				TraceError(
					TRACE_DEVICE,
					"WdfDeviceAllocAndQueryPropertyEx failed with status %!STATUS!",
					status
				);
				break;
			}

			//
			// Convert hex string into UINT64
			// 
			UINT64 hostAddress = wcstoull(
				WdfMemoryGetBuffer(deviceAddressMemory, NULL),
				L'\0',
				16
			);

			WdfObjectDelete(deviceAddressMemory);

			//
			// Convert to MAC address type
			// 

			pDevCtx->DeviceAddress.Address[0] = (UCHAR)((hostAddress >> (8 * 0)) & 0xFF);
			pDevCtx->DeviceAddress.Address[1] = (UCHAR)((hostAddress >> (8 * 1)) & 0xFF);
			pDevCtx->DeviceAddress.Address[2] = (UCHAR)((hostAddress >> (8 * 2)) & 0xFF);
			pDevCtx->DeviceAddress.Address[3] = (UCHAR)((hostAddress >> (8 * 3)) & 0xFF);
			pDevCtx->DeviceAddress.Address[4] = (UCHAR)((hostAddress >> (8 * 4)) & 0xFF);
			pDevCtx->DeviceAddress.Address[5] = (UCHAR)((hostAddress >> (8 * 5)) & 0xFF);

			TraceVerbose(
				TRACE_DEVICE,
				"Device address: %012llX",
				*(PULONGLONG)&pDevCtx->DeviceAddress
			);

			WDF_DEVICE_PROPERTY_DATA_INIT(&devProp, &DEVPKEY_Bluetooth_DeviceVID);

			status = WdfDeviceQueryPropertyEx(
				Device,
				&devProp,
				sizeof(USHORT),
				&pDevCtx->VendorId,
				&requiredSize,
				&propType
			);
			if (!NT_SUCCESS(status))
			{
				TraceError(
					TRACE_DEVICE,
					"Requesting DEVPKEY_Bluetooth_DeviceVID failed with %!STATUS!",
					status
				);
				break;
			}

			TraceVerbose(TRACE_DEVICE, "[BTH] VID: 0x%04X", pDevCtx->VendorId);

			WDF_DEVICE_PROPERTY_DATA_INIT(&devProp, &DEVPKEY_Bluetooth_DevicePID);

			status = WdfDeviceQueryPropertyEx(
				Device,
				&devProp,
				sizeof(USHORT),
				&pDevCtx->ProductId,
				&requiredSize,
				&propType
			);
			if (!NT_SUCCESS(status))
			{
				TraceError(
					TRACE_DEVICE,
					"Requesting DEVPKEY_Bluetooth_DevicePID failed with %!STATUS!",
					status
				);
				break;
			}

			TraceVerbose(TRACE_DEVICE, "[BTH] PID: 0x%04X", pDevCtx->ProductId);

			DsDevice_RegisterBthDisconnectListener(pDevCtx);
			
			DsDevice_RegisterHotReloadListener(pDevCtx);
		}
	} while (FALSE);

	FuncExit(TRACE_DEVICE, "status=%!STATUS!", status);
	
	return status;
}

//
// Gets invoked when the hot-reload event got triggered from somewhere
// 
VOID CALLBACK 
DsDevice_HotRealodEventCallback(
	_In_ PVOID   lpParameter,
	_In_ BOOLEAN TimerOrWaitFired
) 
{
	PDEVICE_CONTEXT pDevCtx = (PDEVICE_CONTEXT)lpParameter;
	UNREFERENCED_PARAMETER(TimerOrWaitFired);

	DsDevice_HotReloadConfiguration(pDevCtx);
}

//
// Read device properties which can be refreshed during runtime
//
VOID DsDevice_HotReloadConfiguration(PDEVICE_CONTEXT Context)
{
	WDF_DEVICE_PROPERTY_DATA propData;
	DEVPROPTYPE propType;
	ULONG requiredSize = 0;
	WDFDEVICE device = WdfObjectContextGetObject(Context);

	TraceVerbose(
		TRACE_DEVICE,
		"Hot-reload triggered"
	);

	WDF_DEVICE_PROPERTY_DATA_INIT(&propData, &DEVPKEY_DsHidMini_HR_MuteDigitalPressureButtons);

	(void)WdfDeviceQueryPropertyEx(
		device,
		&propData,
		sizeof(UCHAR),
		&Context->Configuration.MuteDigitalPressureButtons,
		&requiredSize,
		&propType
	);
}

//
// Reads variable settings properties. Keep in sync with <dshmguid.h> and <dshidmini.inf>
// 
VOID DsDevice_ReadConfiguration(WDFDEVICE Device)
{
	PDEVICE_CONTEXT pDevCtx = DeviceGetContext(Device);
	WDF_DEVICE_PROPERTY_DATA propertyData;
	ULONG requiredSize = 0;
	DEVPROPTYPE propertyType;

	FuncEntry(TRACE_DEVICE);
	
	WDF_DEVICE_PROPERTY_DATA_INIT(&propertyData, &DEVPKEY_DsHidMini_RW_HidDeviceMode);

	(void)WdfDeviceQueryPropertyEx(
		Device,
		&propertyData,
		sizeof(UCHAR),
		&pDevCtx->Configuration.HidDeviceMode,
		&requiredSize,
		&propertyType
	);

	TraceVerbose(TRACE_DEVICE, "[COM] HidDeviceMode: 0x%02X",
		pDevCtx->Configuration.HidDeviceMode);

	
	WDF_DEVICE_PROPERTY_DATA_INIT(&propertyData, &DEVPKEY_DsHidMini_RW_IsOutputRateControlEnabled);

	(void)WdfDeviceQueryPropertyEx(
		Device,
		&propertyData,
		sizeof(UCHAR),
		&pDevCtx->Configuration.IsOutputRateControlEnabled,
		&requiredSize,
		&propertyType
	);

	TraceVerbose(TRACE_DEVICE, "[COM] IsOutputRateControlEnabled: %d",
		pDevCtx->Configuration.IsOutputRateControlEnabled);

	
	WDF_DEVICE_PROPERTY_DATA_INIT(&propertyData, &DEVPKEY_DsHidMini_RW_OutputRateControlPeriodMs);

	(void)WdfDeviceQueryPropertyEx(
		Device,
		&propertyData,
		sizeof(UCHAR),
		&pDevCtx->Configuration.OutputRateControlPeriodMs,
		&requiredSize,
		&propertyType
	);

	TraceVerbose(TRACE_DEVICE, "[COM] OutputRateControlPeriodMs: %d",
		pDevCtx->Configuration.OutputRateControlPeriodMs);

	
	WDF_DEVICE_PROPERTY_DATA_INIT(&propertyData, &DEVPKEY_DsHidMini_RW_IsOutputDeduplicatorEnabled);

	(void)WdfDeviceQueryPropertyEx(
		Device,
		&propertyData,
		sizeof(UCHAR),
		&pDevCtx->Configuration.IsOutputDeduplicatorEnabled,
		&requiredSize,
		&propertyType
	);

	TraceVerbose(TRACE_DEVICE, "[COM] IsOutputDeduplicatorEnabled: %d",
		pDevCtx->Configuration.IsOutputDeduplicatorEnabled);


	WDF_DEVICE_PROPERTY_DATA_INIT(&propertyData, &DEVPKEY_DsHidMini_RW_WirelessIdleTimeoutPeriodMs);

	(void)WdfDeviceQueryPropertyEx(
		Device,
		&propertyData,
		sizeof(ULONG),
		&pDevCtx->Configuration.WirelessIdleTimeoutPeriodMs,
		&requiredSize,
		&propertyType
	);

	TraceVerbose(TRACE_DEVICE, "[COM] WirelessIdleTimeoutPeriodMs: %d",
		pDevCtx->Configuration.WirelessIdleTimeoutPeriodMs);

	WDF_DEVICE_PROPERTY_DATA_INIT(&propertyData, &DEVPKEY_DsHidMini_RW_LargeRumbleDeadzone);

	(void)WdfDeviceQueryPropertyEx(
		Device,
		&propertyData,
		sizeof(UCHAR),
		&pDevCtx->Configuration.LargeRumbleDeadzone,
		&requiredSize,
		&propertyType
	);

	TraceVerbose(TRACE_DEVICE, "[COM] LargeRumbleDeadzone: %d",
		pDevCtx->Configuration.LargeRumbleDeadzone);

	WDF_DEVICE_PROPERTY_DATA_INIT(&propertyData, &DEVPKEY_DsHidMini_RW_SmallRumbleThreshold);

	(void)WdfDeviceQueryPropertyEx(
		Device,
		&propertyData,
		sizeof(ULONG),
		&pDevCtx->Configuration.SmallRumbleThreshold,
		&requiredSize,
		&propertyType
	);

	TraceVerbose(TRACE_DEVICE, "[COM] SmallRumbleThreshold: %d",
		pDevCtx->Configuration.SmallRumbleThreshold);

	WDF_DEVICE_PROPERTY_DATA_INIT(&propertyData, &DEVPKEY_DsHidMini_RW_SmallRumbleDiversion);

	(void)WdfDeviceQueryPropertyEx(
		Device,
		&propertyData,
		sizeof(ULONG),
		&pDevCtx->Configuration.SmallRumbleDiversion,
		&requiredSize,
		&propertyType
	);

	TraceVerbose(TRACE_DEVICE, "[COM] SmallRumbleDiversion: %d",
		pDevCtx->Configuration.SmallRumbleDiversion);

	//
	// Read hot-reloadable properties
	//
	DsDevice_HotReloadConfiguration(pDevCtx);

	FuncExitNoReturn(TRACE_DEVICE);
}

//
// Initialize remaining device context fields
// 
NTSTATUS
DsDevice_InitContext(
	WDFDEVICE Device
)
{
	PDEVICE_CONTEXT pDevCtx = DeviceGetContext(Device);
	NTSTATUS status = STATUS_SUCCESS;
	WDF_OBJECT_ATTRIBUTES attributes;
	PUCHAR outReportBuffer = NULL;
	WDF_TIMER_CONFIG timerCfg;
	
	FuncEntry(TRACE_DEVICE);
		
	switch (pDevCtx->ConnectionType)
	{
	case DsDeviceConnectionTypeUsb:

		//
		// Create managed memory object
		// 
		WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
		attributes.ParentObject = Device;
		status = WdfMemoryCreate(
			&attributes,
			NonPagedPoolNx,
			DS3_POOL_TAG,
			DS3_USB_HID_OUTPUT_REPORT_SIZE,
			&pDevCtx->OutputReportMemory,
			(PVOID*)&outReportBuffer
		);
		if (!NT_SUCCESS(status))
		{
			TraceError(
				TRACE_DEVICE,
				"WdfMemoryCreate failed with %!STATUS!",
				status
			);
			
			break;
		}

		//
		// Fill with default report
		// 
		RtlCopyMemory(
			outReportBuffer,
			G_Ds3UsbHidOutputReport,
			DS3_USB_HID_OUTPUT_REPORT_SIZE
		);

		break;

	case DsDeviceConnectionTypeBth:

		//
		// Create managed memory object
		// 
		WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
		attributes.ParentObject = Device;
		status = WdfMemoryCreate(
			&attributes,
			NonPagedPoolNx,
			DS3_POOL_TAG,
			DS3_USB_HID_OUTPUT_REPORT_SIZE,
			&pDevCtx->OutputReportMemory,
			(PVOID*)&outReportBuffer
		);
		if (!NT_SUCCESS(status))
		{
			TraceError(
				TRACE_DEVICE,
				"WdfMemoryCreate failed with %!STATUS!",
				status
			);
			
			break;
		}

		//
		// Fill with default report
		// 
		RtlCopyMemory(
			outReportBuffer,
			G_Ds3BthHidOutputReport,
			DS3_BTH_HID_OUTPUT_REPORT_SIZE
		);

		//
		// Turn flashing LEDs off
		// 
		DS3_BTH_SET_LED(outReportBuffer, DS3_LED_OFF);

		//
		// Output Report Delay
		// 

		WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
		attributes.ParentObject = Device;

		WDF_TIMER_CONFIG_INIT(
			&timerCfg,
			DsBth_EvtControlWriteTimerFunc
		);

		status = WdfTimerCreate(
			&timerCfg,
			&attributes,
			&pDevCtx->Connection.Bth.Timers.HidOutputReport
		);
		if (!NT_SUCCESS(status))
		{
			TraceError(
				TRACE_DSBTH,
				"WdfTimerCreate (HidOutputReport) failed with status %!STATUS!",
				status
			);
			
			break;
		}

		break;
	}

	do
	{
		if (!NT_SUCCESS(status))
		{
			break;
		}
		
		//
		// Create lock
		// 

		WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
		attributes.ParentObject = Device;

		status = WdfWaitLockCreate(
			&attributes,
			&pDevCtx->OutputReport.Lock
		);
		if (!NT_SUCCESS(status))
		{
			TraceError(
				TRACE_DEVICE,
				"WdfWaitLockCreate failed with status %!STATUS!",
				status
			);
			break;
		}

		//
		// Create lock
		// 

		WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
		attributes.ParentObject = Device;

		status = WdfWaitLockCreate(
			&attributes,
			&pDevCtx->OutputReport.Cache.Lock
		);
		if (!NT_SUCCESS(status))
		{
			TraceError(
				TRACE_DEVICE,
				"WdfWaitLockCreate failed with status %!STATUS!",
				status
			);
			break;
		}

		//
		// Create timer
		// 

		WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
		attributes.ParentObject = Device;

		WDF_TIMER_CONFIG_INIT(
			&timerCfg,
			DSHM_OutputReportDelayTimerElapsed
		);

		status = WdfTimerCreate(
			&timerCfg,
			&attributes,
			&pDevCtx->OutputReport.Cache.SendDelayTimer
		);
		if (!NT_SUCCESS(status))
		{
			TraceError(
				TRACE_DEVICE,
				"WdfTimerCreate failed with status %!STATUS!",
				status
			);
			break;
		}
	}
	while (FALSE);
	
	FuncExit(TRACE_DEVICE, "status=%!STATUS!", status);
	
	return status;
}

//
// Checks if this device is a USB device
// 
NTSTATUS
DsDevice_IsUsbDevice(
	PWDFDEVICE_INIT DeviceInit, 
	PBOOLEAN Result
)
{
	NTSTATUS status;
	WCHAR enumeratorName[200];
	ULONG returnSize;
	UNICODE_STRING unicodeEnumName, temp;

	status = WdfFdoInitQueryProperty(
		DeviceInit,
		DevicePropertyEnumeratorName,
		sizeof(enumeratorName),
		enumeratorName,
		&returnSize
	);
	if (!NT_SUCCESS(status))
	{
		return status;
	}

	RtlInitUnicodeString(
		&unicodeEnumName,
		enumeratorName
	);

	RtlInitUnicodeString(
		&temp,
		L"USB"
	);

	if (Result)
		*Result = RtlCompareUnicodeString(&unicodeEnumName, &temp, TRUE) == 0;

	return status;
}

/**
 * Registers an event listener to trigger refreshing runtime properties
 *
 * @author	Benjamin "Nefarius" H�glinger-Stelzer
 * @date	15.04.2021
 *
 * @param 	Context	The context.
 */
void DsDevice_RegisterHotReloadListener(PDEVICE_CONTEXT Context)
{
	WCHAR eventName[49];
	WCHAR deviceAddress[13];

	FuncEntry(TRACE_DEVICE);

	switch (Context->ConnectionType)
	{
	case DsDeviceConnectionTypeUsb:

		swprintf_s(
			deviceAddress,
			ARRAYSIZE(deviceAddress),
			L"%02X%02X%02X%02X%02X%02X",
			Context->DeviceAddress.Address[0],
			Context->DeviceAddress.Address[1],
			Context->DeviceAddress.Address[2],
			Context->DeviceAddress.Address[3],
			Context->DeviceAddress.Address[4],
			Context->DeviceAddress.Address[5]
		);

		break;
	case DsDeviceConnectionTypeBth:

		swprintf_s(
			deviceAddress,
			ARRAYSIZE(deviceAddress),
			L"%012llX",
			*(PULONGLONG)&Context->DeviceAddress
		);

		break;
	}

	//
	// Register global event to listen for changes on
	//

	swprintf_s(
		eventName,
		ARRAYSIZE(eventName),
		L"Global\\DsHidMiniConfigHotReloadEvent%ls",
		deviceAddress
	);

	TraceVerbose(
		TRACE_DEVICE,
		"Configuration reload event name: %ls",
		eventName
	);

	TCHAR* szSD = TEXT("D:(A;OICI;GA;;;BA)(A;OICI;GA;;;SY)");
	SECURITY_ATTRIBUTES sa;
	sa.nLength = sizeof(SECURITY_ATTRIBUTES);
	sa.bInheritHandle = FALSE;
	ConvertStringSecurityDescriptorToSecurityDescriptor(
		szSD,
		SDDL_REVISION_1,
		&((&sa)->lpSecurityDescriptor),
		NULL
	);

	if (Context->ConfigurationReloadEvent)
	{
		CloseHandle(Context->ConfigurationReloadEvent);
		Context->ConfigurationReloadEvent = NULL;
	}

	Context->ConfigurationReloadEvent = CreateEventW(
		&sa,
		FALSE,
		FALSE,
		eventName
	);

	if (Context->ConfigurationReloadEvent == NULL)
	{
		TraceError(
			TRACE_DEVICE,
			"Failed to create reload event"
		);
	}

	const BOOL ret = RegisterWaitForSingleObject(
		&Context->ConfigurationReloadWaitHandle,
		Context->ConfigurationReloadEvent,
		DsDevice_HotRealodEventCallback,
		Context,
		INFINITE,
		WT_EXECUTELONGFUNCTION
	);

	if (!ret)
	{
		TraceError(
			TRACE_DEVICE,
			"Failed to register wait for reload event"
		);
	}

	FuncExitNoReturn(TRACE_DEVICE);
}

/**
 * Register event to disconnect from Bluetooth, bypassing mshudumdf.sys
 *
 * @author	Benjamin "Nefarius" H�glinger-Stelzer
 * @date	15.04.2021
 *
 * @param 	Context	The context.
 */
void DsDevice_RegisterBthDisconnectListener(PDEVICE_CONTEXT Context)
{
	WCHAR dcEventName[44];
	WCHAR deviceAddress[13];

	FuncEntry(TRACE_DEVICE);

	swprintf_s(
		deviceAddress,
		ARRAYSIZE(deviceAddress),
		L"%012llX",
		*(PULONGLONG)&Context->DeviceAddress
	);
	
	swprintf_s(
		dcEventName,
		ARRAYSIZE(dcEventName),
		DSHM_NAMED_EVENT_DISCONNECT,
		deviceAddress
	);

	TraceVerbose(
		TRACE_DEVICE,
		"Disconnect event name: %ls",
		dcEventName
	);

	TCHAR* szSD = TEXT("D:(A;;0x001F0003;;;BA)(A;;0x00100002;;;AU)");
	SECURITY_ATTRIBUTES sa;
	sa.nLength = sizeof(SECURITY_ATTRIBUTES);
	sa.bInheritHandle = FALSE;
	ConvertStringSecurityDescriptorToSecurityDescriptor(
		szSD,
		SDDL_REVISION_1,
		&((&sa)->lpSecurityDescriptor),
		NULL
	);

	if (Context->Connection.Bth.DisconnectEvent) {
		CloseHandle(Context->Connection.Bth.DisconnectEvent);
		Context->Connection.Bth.DisconnectEvent = NULL;
	}

	Context->Connection.Bth.DisconnectEvent = CreateEventW(
		&sa,
		FALSE,
		FALSE,
		dcEventName
	);

	if (Context->Connection.Bth.DisconnectEvent == NULL)
	{
		TraceError(
			TRACE_DEVICE,
			"Failed to create disconnect event"
		);
	}

	const BOOL ret = RegisterWaitForSingleObject(
		&Context->Connection.Bth.DisconnectWaitHandle,
		Context->Connection.Bth.DisconnectEvent,
		DsBth_DisconnectEventCallback,
		Context,
		INFINITE,
		WT_EXECUTELONGFUNCTION | WT_EXECUTEONLYONCE
	);

	if (!ret)
	{
		TraceError(
			TRACE_DEVICE,
			"Failed to register wait for disconnect event"
		);
	}

	FuncExitNoReturn(TRACE_DEVICE);
}

/**
 * Signals existing wireless connection with same device address to terminate. The controller
 * does not disconnect from Bluetooth on its own once connected to USB, so we signal the
 * wireless device object to disconnect itself before continuing with USB initialization.
 *
 * @author	Benjamin "Nefarius" H�glinger-Stelzer
 * @date	15.04.2021
 *
 * @param 	Context	The context.
 */
void DsDevice_InvokeLocalBthDisconnect(PDEVICE_CONTEXT Context)
{
	WCHAR deviceAddress[13];
	WCHAR dcEventName[44];

	//
	// Convert to expected hex string
	// 
	swprintf_s(
		deviceAddress,
		ARRAYSIZE(deviceAddress),
		L"%02X%02X%02X%02X%02X%02X",
		Context->DeviceAddress.Address[0],
		Context->DeviceAddress.Address[1],
		Context->DeviceAddress.Address[2],
		Context->DeviceAddress.Address[3],
		Context->DeviceAddress.Address[4],
		Context->DeviceAddress.Address[5]
	);

	//
	// Disconnect Bluetooth connection, if detected
	//

	swprintf_s(
		dcEventName,
		ARRAYSIZE(dcEventName),
		DSHM_NAMED_EVENT_DISCONNECT,
		deviceAddress
	);

	HANDLE dcEvent = OpenEventW(
		SYNCHRONIZE | EVENT_MODIFY_STATE,
		FALSE,
		dcEventName
	);

	if (dcEvent != NULL)
	{
		TraceVerbose(
			TRACE_DSUSB,
			"Found existing event %ls, signalling disconnect",
			dcEventName
		);

		SetEvent(dcEvent);
		CloseHandle(dcEvent);
	}
	else
	{
		TraceError(
			TRACE_DSUSB,
			"GetLastError: %d",
			GetLastError()
		);
	}
}

//
// Bootstrap required DMF modules
// 
#pragma code_seg("PAGED")
_IRQL_requires_max_(PASSIVE_LEVEL)
VOID
DmfDeviceModulesAdd(
	_In_ WDFDEVICE Device,
	_In_ PDMFMODULE_INIT DmfModuleInit
)
{
	PDEVICE_CONTEXT pDevCtx;
	DMF_MODULE_ATTRIBUTES moduleAttributes;
	DMF_CONFIG_DsHidMini dsHidMiniCfg;
	DMF_CONFIG_ThreadedBufferQueue dmfBufferCfg;
	DMF_CONFIG_DefaultTarget bthReaderCfg;
	DMF_CONFIG_DefaultTarget bthWriterCfg;
	
	PAGED_CODE();

	FuncEntry(TRACE_DEVICE);

	pDevCtx = DeviceGetContext(Device);

	//
	// Threaded buffer queue used to serialize output report packets
	// 

	DMF_CONFIG_ThreadedBufferQueue_AND_ATTRIBUTES_INIT(
		&dmfBufferCfg,
		&moduleAttributes
	);
	moduleAttributes.PassiveLevel = TRUE;

	dmfBufferCfg.EvtThreadedBufferQueueWork = DMF_EvtExecuteOutputPacketReceived;
	// Fixed amount of buffers, no auto-grow
	dmfBufferCfg.BufferQueueConfig.SourceSettings.EnableLookAside = FALSE;
	/*
	 * TODO: tune to find good value
	 * - too low: packets might get dropped unintentionally
	 * - too high: user-noticeable delay may build up
	 */
	dmfBufferCfg.BufferQueueConfig.SourceSettings.BufferCount = 10;
	dmfBufferCfg.BufferQueueConfig.SourceSettings.BufferSize = DS3_BTH_HID_OUTPUT_REPORT_SIZE;
	dmfBufferCfg.BufferQueueConfig.SourceSettings.BufferContextSize = sizeof(DS_OUTPUT_REPORT_CONTEXT);
	dmfBufferCfg.BufferQueueConfig.SourceSettings.PoolType = NonPagedPoolNx;

	DMF_DmfModuleAdd(
		DmfModuleInit,
		&moduleAttributes,
		WDF_NO_OBJECT_ATTRIBUTES,
		&pDevCtx->OutputReport.Worker
	);

	//
	// Avoid allocating modules not used on USB
	// 
	if (pDevCtx->ConnectionType == DsDeviceConnectionTypeBth)
	{
		//
		// Default I/O target request streamer for input reports
		// 

		DMF_CONFIG_DefaultTarget_AND_ATTRIBUTES_INIT(
			&bthReaderCfg,
			&moduleAttributes
		);
		moduleAttributes.PassiveLevel = TRUE;
		
		bthReaderCfg.ContinuousRequestTargetModuleConfig.BufferCountOutput = 1;
		bthReaderCfg.ContinuousRequestTargetModuleConfig.BufferOutputSize = BTHPS3_SIXAXIS_HID_INPUT_REPORT_SIZE;
		bthReaderCfg.ContinuousRequestTargetModuleConfig.ContinuousRequestCount = 1;
		bthReaderCfg.ContinuousRequestTargetModuleConfig.PoolTypeOutput = NonPagedPoolNx;
		bthReaderCfg.ContinuousRequestTargetModuleConfig.PurgeAndStartTargetInD0Callbacks = FALSE;
		bthReaderCfg.ContinuousRequestTargetModuleConfig.ContinuousRequestTargetIoctl = IOCTL_BTHPS3_HID_INTERRUPT_READ;
		bthReaderCfg.ContinuousRequestTargetModuleConfig.EvtContinuousRequestTargetBufferOutput = DsBth_HidInterruptReadContinuousRequestCompleted;
		bthReaderCfg.ContinuousRequestTargetModuleConfig.RequestType = ContinuousRequestTarget_RequestType_Ioctl;
		bthReaderCfg.ContinuousRequestTargetModuleConfig.ContinuousRequestTargetMode = ContinuousRequestTarget_Mode_Manual;
		
		DMF_DmfModuleAdd(
			DmfModuleInit,
			&moduleAttributes,
			WDF_NO_OBJECT_ATTRIBUTES,
			&pDevCtx->Connection.Bth.HidInterrupt.InputStreamerModule
		);

		
		DMF_CONFIG_DefaultTarget_AND_ATTRIBUTES_INIT(
			&bthWriterCfg,
			&moduleAttributes
		);
		moduleAttributes.PassiveLevel = TRUE;

		bthWriterCfg.ContinuousRequestTargetModuleConfig.BufferCountInput = 1;
		bthWriterCfg.ContinuousRequestTargetModuleConfig.BufferInputSize = BTHPS3_SIXAXIS_HID_OUTPUT_REPORT_SIZE;
		bthWriterCfg.ContinuousRequestTargetModuleConfig.ContinuousRequestCount = 1;
		bthWriterCfg.ContinuousRequestTargetModuleConfig.PoolTypeInput = NonPagedPoolNx;
		bthWriterCfg.ContinuousRequestTargetModuleConfig.PurgeAndStartTargetInD0Callbacks = FALSE;
		bthWriterCfg.ContinuousRequestTargetModuleConfig.ContinuousRequestTargetIoctl = IOCTL_BTHPS3_HID_CONTROL_WRITE;
		bthWriterCfg.ContinuousRequestTargetModuleConfig.EvtContinuousRequestTargetBufferInput = DsBth_HidControlWriteContinuousRequestCompleted;
		bthWriterCfg.ContinuousRequestTargetModuleConfig.RequestType = ContinuousRequestTarget_RequestType_Ioctl;
		bthWriterCfg.ContinuousRequestTargetModuleConfig.ContinuousRequestTargetMode = ContinuousRequestTarget_Mode_Manual;

		DMF_DmfModuleAdd(
			DmfModuleInit,
			&moduleAttributes,
			WDF_NO_OBJECT_ATTRIBUTES,
			&pDevCtx->Connection.Bth.HidControl.OutputWriterModule
		);
	}
	
	//
	// Virtual HID Mini Module
	// 
	
	DMF_CONFIG_DsHidMini_AND_ATTRIBUTES_INIT(
		&dsHidMiniCfg,
		&moduleAttributes
	);

	DMF_DmfModuleAdd(
		DmfModuleInit,
		&moduleAttributes,
		WDF_NO_OBJECT_ATTRIBUTES,
		&pDevCtx->DsHidMiniModule
	);

	FuncExitNoReturn(TRACE_DEVICE);
}
#pragma code_seg()

#pragma region I/O Queue Callbacks

void DSHM_EvtWdfIoQueueIoDeviceControl(
	WDFQUEUE Queue,
	WDFREQUEST Request,
	size_t OutputBufferLength,
	size_t InputBufferLength,
	ULONG IoControlCode
)
{
	NTSTATUS status = STATUS_NOT_IMPLEMENTED;

	UNREFERENCED_PARAMETER(Queue);
	UNREFERENCED_PARAMETER(OutputBufferLength);
	UNREFERENCED_PARAMETER(InputBufferLength);

	FuncEntry(TRACE_DEVICE);

	switch (IoControlCode)
	{
	case IOCTL_HID_DEVICERESET_NOTIFICATION:
		TraceVerbose(
			TRACE_DEVICE,
			"IOCTL_HID_DEVICERESET_NOTIFICATION not supported"
		);
		status = STATUS_NOT_SUPPORTED;
		break;
	default:
		TraceVerbose(TRACE_DEVICE, "Unhandled I/O control code 0x%X", IoControlCode);
		break;
	}

	FuncExitNoReturn(TRACE_DEVICE);

	WdfRequestComplete(Request, status);
}

#pragma endregion
