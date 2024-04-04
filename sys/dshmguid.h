#pragma once

//
// Interface GUID used to identify/enumerate devices running under the driver
// 

// {399ED672-E0BD-4FB3-AB0C-4955B56FB86A}
DEFINE_GUID(GUID_DEVINTERFACE_DSHIDMINI,
	0x399ed672, 0xe0bd, 0x4fb3, 0xab, 0xc, 0x49, 0x55, 0xb5, 0x6f, 0xb8, 0x6a);


#pragma region Read-only properties

//
// Use this category for status/state reporting only
// Driver writes during operation
// Applications read only
// 

// {3FECF510-CC94-4FBE-8839-738201F84D59}
DEFINE_DEVPROPKEY(DEVPKEY_DsHidMini_RO_BatteryStatus,
	0x3fecf510, 0xcc94, 0x4fbe, 0x88, 0x39, 0x73, 0x82, 0x1, 0xf8, 0x4d, 0x59, 2); // DEVPROP_TYPE_BYTE

// {3FECF510-CC94-4FBE-8839-738201F84D59}
DEFINE_DEVPROPKEY(DEVPKEY_DsHidMini_RO_LastPairingStatus,
	0x3fecf510, 0xcc94, 0x4fbe, 0x88, 0x39, 0x73, 0x82, 0x1, 0xf8, 0x4d, 0x59, 3); // DEVPROP_TYPE_NTSTATUS

// {3FECF510-CC94-4FBE-8839-738201F84D59}
DEFINE_DEVPROPKEY(DEVPKEY_DsHidMini_RO_IdentificationData,
	0x3fecf510, 0xcc94, 0x4fbe, 0x88, 0x39, 0x73, 0x82, 0x1, 0xf8, 0x4d, 0x59, 4); // DEVPROP_TYPE_BINARY

#pragma endregion

#pragma region Boot configuration properties

//
// Use this category for variable settings
// Can be changed by applications
// Driver reads only once on power-up
// 

// {6D293077-C3D6-4062-9597-BE4389404C02}
DEFINE_DEVPROPKEY(DEVPKEY_DsHidMini_RW_HidDeviceMode,
	0x6d293077, 0xc3d6, 0x4062, 0x95, 0x97, 0xbe, 0x43, 0x89, 0x40, 0x4c, 0x2, 2); // DEVPROP_TYPE_BYTE

// {6D293077-C3D6-4062-9597-BE4389404C02}
DEFINE_DEVPROPKEY(DEVPKEY_DsHidMini_RW_IsOutputRateControlEnabled,
	0x6d293077, 0xc3d6, 0x4062, 0x95, 0x97, 0xbe, 0x43, 0x89, 0x40, 0x4c, 0x2, 3); // DEVPROP_TYPE_BYTE

// {6D293077-C3D6-4062-9597-BE4389404C02}
DEFINE_DEVPROPKEY(DEVPKEY_DsHidMini_RW_OutputRateControlPeriodMs,
	0x6d293077, 0xc3d6, 0x4062, 0x95, 0x97, 0xbe, 0x43, 0x89, 0x40, 0x4c, 0x2, 4); // DEVPROP_TYPE_BYTE

// {6D293077-C3D6-4062-9597-BE4389404C02}
DEFINE_DEVPROPKEY(DEVPKEY_DsHidMini_RW_IsOutputDeduplicatorEnabled,
	0x6d293077, 0xc3d6, 0x4062, 0x95, 0x97, 0xbe, 0x43, 0x89, 0x40, 0x4c, 0x2, 5); // DEVPROP_TYPE_BYTE

// {6D293077-C3D6-4062-9597-BE4389404C02}
DEFINE_DEVPROPKEY(DEVPKEY_DsHidMini_RW_WirelessIdleTimeoutPeriodMs,
	0x6d293077, 0xc3d6, 0x4062, 0x95, 0x97, 0xbe, 0x43, 0x89, 0x40, 0x4c, 0x2, 6); // DEVPROP_TYPE_UINT32

// {6D293077-C3D6-4062-9597-BE4389404C02}
DEFINE_DEVPROPKEY(DEVPKEY_DsHidMini_RW_LargeRumbleDeadzone,
	0x6d293077, 0xc3d6, 0x4062, 0x95, 0x97, 0xbe, 0x43, 0x89, 0x40, 0x4c, 0x2, 7); // DEVPROP_TYPE_BYTE

// {6D293077-C3D6-4062-9597-BE4389404C02}
DEFINE_DEVPROPKEY(DEVPKEY_DsHidMini_RW_SmallRumbleThreshold,
	0x6d293077, 0xc3d6, 0x4062, 0x95, 0x97, 0xbe, 0x43, 0x89, 0x40, 0x4c, 0x2, 8); // DEVPROP_TYPE_BYTE

// {6D293077-C3D6-4062-9597-BE4389404C02}
DEFINE_DEVPROPKEY(DEVPKEY_DsHidMini_RW_SmallRumbleDiversion,
	0x6d293077, 0xc3d6, 0x4062, 0x95, 0x97, 0xbe, 0x43, 0x89, 0x40, 0x4c, 0x2, 9); // DEVPROP_TYPE_BYTE

#pragma endregion

#pragma region Hot-reload changeable properties

//
// Use this category for variable settings which may change during operation
// Can be changed by applications at any time
// Driver refreshes value when reload event is triggered
// 

// {99ACCB6C-D709-49BB-90EE-278B1B564A4B}
DEFINE_DEVPROPKEY(DEVPKEY_DsHidMini_HR_MuteDigitalPressureButtons,
	0x99accb6c, 0xd709, 0x49bb, 0x90, 0xee, 0x27, 0x8b, 0x1b, 0x56, 0x4a, 0x4b, 2); // DEVPROP_TYPE_BYTE

#pragma endregion
