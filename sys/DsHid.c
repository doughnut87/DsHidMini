#include <math.h>
#include "Driver.h"
#include "DsHid.tmh"
#ifdef DSHM_FEATURE_FFB
#include "PID/PIDTypes.h"
#endif
#ifndef M_PI
#    define M_PI 3.1415926535897932
#endif

#pragma region DS3 HID Report Descriptor (Split Device Mode)

CONST HID_REPORT_DESCRIPTOR G_Ds3HidReportDescriptor_Split_Mode[] = 
{
	/************************************************************************/
	/* Gamepad definition (for regular DS3 buttons, axes & features)        */
	/************************************************************************/
#include "HID/02_GPJ_Col1_Gamepad.h"
#ifdef DSHM_FEATURE_FFB
#include "PID/01_PIDStateReport.h"
#include "PID/02_SetEffectReport.h"
#include "PID/03_SetEnvelopeReport.h"
#include "PID/04_SetConditionReport.h"
#include "PID/05_SetPeriodicReport.h"
#include "PID/06_SetConstantForceReport.h"
#include "PID/07_SetRampForceReport.h"
#include "PID/08_CustomForceDataReport.h"
#include "PID/09_DownloadForceSample.h"
#include "PID/10_EffectOperationReport.h"
#include "PID/11_PIDBlockFreeReport.h"
#include "PID/12_PIDDeviceControl.h"
#include "PID/13_DeviceGainReport.h"
#include "PID/14_SetCustomForceReport.h"
#include "PID/15_CreateNewEffectReport.h"
#include "PID/16_PIDBlockLoadReport.h"
#include "PID/17_PIDPoolReport.h"
#endif
	0xC0,              // End Collection
	/************************************************************************/
	/* Joystick definition (for exposing pressure values as axes)           */
	/************************************************************************/
#include "HID/02_GPJ_Col2_Joystick.h"
};

CONST HID_DESCRIPTOR G_Ds3HidDescriptor_Split_Mode = {
	0x09,   // length of HID descriptor
	0x21,   // descriptor type == HID  0x21
	0x0100, // hid spec release
	0x00,   // country code == Not Specified
	0x01,   // number of HID class descriptors
{ 0x22,   // descriptor type 
sizeof(G_Ds3HidReportDescriptor_Split_Mode) }  // total length of report descriptor
};

#pragma endregion

#pragma region DS3 HID Report Descriptor (Single Device Mode)

CONST HID_REPORT_DESCRIPTOR G_Ds3HidReportDescriptor_Single_Mode[] = 
{
	/************************************************************************/
	/* Gamepad definition with pressure axes in one report                  */
	/************************************************************************/
#include "HID/01_SDF_Col1_GamePad.h"
#ifdef DSHM_FEATURE_FFB
#include "PID/01_PIDStateReport.h"
#include "PID/02_SetEffectReport.h"
#include "PID/03_SetEnvelopeReport.h"
#include "PID/04_SetConditionReport.h"
#include "PID/05_SetPeriodicReport.h"
#include "PID/06_SetConstantForceReport.h"
#include "PID/07_SetRampForceReport.h"
#include "PID/08_CustomForceDataReport.h"
#include "PID/09_DownloadForceSample.h"
#include "PID/10_EffectOperationReport.h"
#include "PID/11_PIDBlockFreeReport.h"
#include "PID/12_PIDDeviceControl.h"
#include "PID/13_DeviceGainReport.h"
#include "PID/14_SetCustomForceReport.h"
#include "PID/15_CreateNewEffectReport.h"
#include "PID/16_PIDBlockLoadReport.h"
#include "PID/17_PIDPoolReport.h"
#endif
	0xC0,              // End Collection
};

CONST HID_DESCRIPTOR G_Ds3HidDescriptor_Single_Mode = {
	0x09,   // length of HID descriptor
	0x21,   // descriptor type == HID  0x21
	0x0100, // hid spec release
	0x00,   // country code == Not Specified
	0x01,   // number of HID class descriptors
{ 0x22,   // descriptor type 
sizeof(G_Ds3HidReportDescriptor_Single_Mode) }  // total length of report descriptor
};

#pragma endregion

#pragma region DS3 HID Report Descriptor (SIXAXIS.SYS compatible)

CONST HID_REPORT_DESCRIPTOR G_SixaxisHidReportDescriptor[] =
{
	/************************************************************************/
	/* SIXAXIS.SYS compatible report descriptor                             */
	/************************************************************************/
#include "HID/03_SXS_Col1_Joystick.h"
};

CONST HID_DESCRIPTOR G_SixaxisHidDescriptor = {
	0x09,   // length of HID descriptor
	0x21,   // descriptor type == HID  0x21
	0x0100, // hid spec release
	0x00,   // country code == Not Specified
	0x01,   // number of HID class descriptors
{ 0x22,   // descriptor type 
sizeof(G_SixaxisHidReportDescriptor) }  // total length of report descriptor
};

#pragma endregion

#pragma region DS3 HID Report Descriptor (Vendor Defined DS4 Rev1 USB emulation)

CONST HID_REPORT_DESCRIPTOR G_VendorDefinedUSBDS4HidReportDescriptor[] =
{
	/************************************************************************/
	/* Vendor Defined DualShock 4 Rev1 USB compatible report descriptor     */
	/************************************************************************/
#include "HID/04_DS4_Col1_VendorDefined.h"
};

CONST HID_DESCRIPTOR G_VendorDefinedUSBDS4HidDescriptor = {
	0x09,   // length of HID descriptor
	0x21,   // descriptor type == HID  0x21
	0x0100, // hid spec release
	0x00,   // country code == Not Specified
	0x01,   // number of HID class descriptors
{ 0x22,   // descriptor type 
sizeof(G_VendorDefinedUSBDS4HidReportDescriptor) }  // total length of report descriptor
};

#pragma endregion

#pragma region DS3 HID Report Descriptor (XINPUT compatible HID device)

CONST HID_REPORT_DESCRIPTOR G_XInputHIDCompatible_HidReportDescriptor[] =
{
#include "HID/05_XIH_Col1_XInputHID.h"
};

CONST HID_DESCRIPTOR G_XInputHIDCompatible_HidDescriptor = {
	0x09,   // length of HID descriptor
	0x21,   // descriptor type == HID  0x21
	0x0100, // hid spec release
	0x00,   // country code == Not Specified
	0x01,   // number of HID class descriptors
{ 0x22,   // descriptor type 
sizeof(G_XInputHIDCompatible_HidReportDescriptor) }  // total length of report descriptor
};

#pragma endregion


BOOLEAN DS3_RAW_IS_IDLE(
	_In_ PDS3_RAW_INPUT_REPORT Input
)
{
	//
	// Button states
	// 

	if (Input->Buttons.lButtons)
	{
		return FALSE;
	}

	//
	// Axes
	// 

	if (
		Input->LeftThumbX < DS3_RAW_AXIS_IDLE_THRESHOLD_LOWER
		|| Input->LeftThumbX > DS3_RAW_AXIS_IDLE_THRESHOLD_UPPER
		|| Input->LeftThumbY < DS3_RAW_AXIS_IDLE_THRESHOLD_LOWER
		|| Input->LeftThumbY > DS3_RAW_AXIS_IDLE_THRESHOLD_UPPER
		|| Input->RightThumbX < DS3_RAW_AXIS_IDLE_THRESHOLD_LOWER
		|| Input->RightThumbX > DS3_RAW_AXIS_IDLE_THRESHOLD_UPPER
		|| Input->RightThumbY < DS3_RAW_AXIS_IDLE_THRESHOLD_LOWER
		|| Input->RightThumbY > DS3_RAW_AXIS_IDLE_THRESHOLD_UPPER
	)
	{
		return FALSE;
	}
	
	//
	// Sliders
	// 

	if (
		Input->Pressure.Values.L2 > DS3_RAW_SLIDER_IDLE_THRESHOLD
		|| Input->Pressure.Values.R2 > DS3_RAW_SLIDER_IDLE_THRESHOLD
	)
	{
		return FALSE;
	}
	
	//
	// If we end up here, no movement is going on
	// 

	return TRUE;
}

VOID DS3_RAW_TO_SPLIT_HID_INPUT_REPORT_01(
	_In_ PDS3_RAW_INPUT_REPORT Input,
	_Out_ PUCHAR Output,
	_In_ BOOLEAN MuteDigitalPressureButtons
)
{
	// Report ID
	Output[0] = 0x01;

	// Prepare D-Pad
	Output[5] &= ~0xF; // Clear lower 4 bits

	// Prepare face buttons
	Output[5] &= ~0xF0; // Clear upper 4 bits

	// Prepare buttons: L2, R2, L1, R1, L3, R3, Select and Start
	Output[6] &= ~0xFF; // Clear all 8 bits

	// Prepare PS and D-Pad buttons
	Output[7] &= ~0xFF; // Clear all 8 bits

	if (!MuteDigitalPressureButtons)
	{
		// Translate D-Pad to HAT format
		if (TRUE == TRUE) // Placeholder for DHMC option
		{
			switch (Input->Buttons.bButtons[0] & ~0xF)
			{
			case 0x10: // N
				Output[5] |= 0 & 0xF;
				break;
			case 0x30: // NE
				Output[5] |= 1 & 0xF;
				break;
			case 0x20: // E
				Output[5] |= 2 & 0xF;
				break;
			case 0x60: // SE
				Output[5] |= 3 & 0xF;
				break;
			case 0x40: // S
				Output[5] |= 4 & 0xF;
				break;
			case 0xC0: // SW
				Output[5] |= 5 & 0xF;
				break;
			case 0x80: // W
				Output[5] |= 6 & 0xF;
				break;
			case 0x90: // NW
				Output[5] |= 7 & 0xF;
				break;
			default: // Released
				Output[5] |= 8 & 0xF;
				break;
			}
		}
		else {
			// Clear HAT position
			Output[5] |= 8 & 0xF;
		}

		// Set face buttons
		Output[5] |= Input->Buttons.bButtons[1] & 0xF0; // OUTPUT: SQUARE [7], CROSS [6], CIRCLE [5], TRIANGLE [4]

		// Remaining buttons
		Output[6] |= (Input->Buttons.bButtons[0] & 0xF); // OUTPUT: START [3], RSB [2], LSB [1], SELECT [0]
		Output[6] |= (Input->Buttons.bButtons[1] & 0xF) << 4; // OUTPUT: R1 [7], L1 [6], R2 [5], L2 [4]

		// D-Pad (Buttons)
		if (FALSE == TRUE) // "FALSE" is a placeholder for the DHMC option that will allow muting the D-Pad Buttons
		{
			Output[7] |= (Input->Buttons.bButtons[0] & ~0xF) >> 3; // OUTPUT: LEFT [4], DOWN [3], RIGHT [2], UP [1]
		}
	}
	else {
		// Clear HAT position
		Output[5] |= 8 & 0xF;
	}

	// PS button
	Output[7] |= Input->Buttons.Individual.PS; // OUTPUT: PS BUTTON [0]

	// Thumb axes
	Output[1] = Input->LeftThumbX;
	Output[2] = Input->LeftThumbY;
	Output[3] = Input->RightThumbX;
	Output[4] = Input->RightThumbY;

	// Trigger axes
	Output[8] = Input->Pressure.Values.L2;
	Output[9] = Input->Pressure.Values.R2;

	// Shoulders (pressure)
	Output[10] = Input->Pressure.Values.L1;
	Output[11] = Input->Pressure.Values.R1;

}

VOID DS3_RAW_TO_SPLIT_HID_INPUT_REPORT_02(
	_In_ PDS3_RAW_INPUT_REPORT Input,
	_Out_ PUCHAR Output
)
{
	// Report ID
	Output[0] = 0x02;

	// D-Pad (pressure)
	Output[1] = Input->Pressure.Values.Up;
	Output[2] = Input->Pressure.Values.Right;
	Output[3] = Input->Pressure.Values.Down;
	Output[4] = Input->Pressure.Values.Left;

	// Face buttons (pressure)
	Output[5] = Input->Pressure.Values.Triangle;
	Output[6] = Input->Pressure.Values.Circle;
	Output[7] = Input->Pressure.Values.Cross;
	Output[8] = Input->Pressure.Values.Square;
}

VOID DS3_RAW_TO_SINGLE_HID_INPUT_REPORT(
	_In_ PDS3_RAW_INPUT_REPORT Input,
	_Out_ PUCHAR Output,
	_In_ BOOLEAN MuteDigitalPressureButtons
)
{
	// Report ID
	Output[0] = Input->ReportId;

	// Prepare D-Pad
	Output[5] &= ~0xF; // Clear lower 4 bits

	// Prepare face buttons
	Output[5] &= ~0xF0; // Clear upper 4 bits

	// Prepare buttons: L2, R2, L1, R1, L3, R3, Select and Start
	Output[6] &= ~0xFF; // Clear all 8 bits

	// Prepare PS and D-Pad buttons
	Output[7] &= ~0xFF; // Clear all 8 bits

	if (!MuteDigitalPressureButtons)
	{
		// Translate D-Pad to HAT format
		if (TRUE == TRUE) // Placeholder for DHMC option
		{
			switch (Input->Buttons.bButtons[0] & ~0xF)
			{
			case 0x10: // N
				Output[5] |= 0 & 0xF;
				break;
			case 0x30: // NE
				Output[5] |= 1 & 0xF;
				break;
			case 0x20: // E
				Output[5] |= 2 & 0xF;
				break;
			case 0x60: // SE
				Output[5] |= 3 & 0xF;
				break;
			case 0x40: // S
				Output[5] |= 4 & 0xF;
				break;
			case 0xC0: // SW
				Output[5] |= 5 & 0xF;
				break;
			case 0x80: // W
				Output[5] |= 6 & 0xF;
				break;
			case 0x90: // NW
				Output[5] |= 7 & 0xF;
				break;
			default: // Released
				Output[5] |= 8 & 0xF;
				break;
			}
		}
		else {
			// Clear HAT position
			Output[5] |= 8 & 0xF;
		}

		// Set face buttons
		Output[5] |= Input->Buttons.bButtons[1] & 0xF0; // OUTPUT: SQUARE[7], CROSS[6], CIRCLE[5], TRIANGLE[4]

		// Remaining buttons
		Output[6] |= (Input->Buttons.bButtons[0] & 0xF);  // OUTPUT: START [3], RSB [2], LSB [1], SELECT [0]
		Output[6] |= (Input->Buttons.bButtons[1] & 0xF) << 4; // OUTPUT: R1 [7], L1 [6], R2 [5], L2 [4]

		// D-Pad (Buttons)
		if (FALSE == TRUE) // "FALSE" is a placeholder for the DHMC option that will allow muting the D-Pad Buttons
		{
			Output[7] |= (Input->Buttons.bButtons[0] & ~0xF) >> 3; // OUTPUT: LEFT [4], DOWN [3], RIGHT [2], UP [1]
		}
	}
	else {
		// Clear HAT position
		Output[5] |= 8 & 0xF;
	}

	// Thumb axes
	Output[1] = Input->LeftThumbX;
	Output[2] = Input->LeftThumbY;
	Output[3] = Input->RightThumbX;
	Output[4] = Input->RightThumbY;

	// Trigger axes
	Output[8] = Input->Pressure.Values.L2;
	Output[9] = Input->Pressure.Values.R2;

	// PS button
	Output[7] |= Input->Buttons.Individual.PS;

	// D-Pad (pressure)
	Output[10] = Input->Pressure.Values.Up;
	Output[11] = Input->Pressure.Values.Right;
	Output[12] = Input->Pressure.Values.Down;
	Output[13] = Input->Pressure.Values.Left;

	// Shoulders (pressure)
	Output[14] = Input->Pressure.Values.L1;
	Output[15] = Input->Pressure.Values.R1;

	// Face buttons (pressure)
	Output[16] = Input->Pressure.Values.Triangle;
	Output[17] = Input->Pressure.Values.Circle;
	Output[18] = Input->Pressure.Values.Cross;
	Output[19] = Input->Pressure.Values.Square;
}

VOID DS3_RAW_TO_SIXAXIS_HID_INPUT_REPORT(
	_In_ PDS3_RAW_INPUT_REPORT Input,
	_Out_ PUCHAR Output
)
{
	// Prepare D-Pad
	Output[3] &= ~0xF; // Clear lower 4 bits

	// Translate D-Pad to HAT format
	switch (Input->Buttons.bButtons[0] & ~0xF)
	{
	case 0x10: // N
		Output[3] |= 0 & 0xF;
		break;
	case 0x30: // NE
		Output[3] |= 1 & 0xF;
		break;
	case 0x20: // E
		Output[3] |= 2 & 0xF;
		break;
	case 0x60: // SE
		Output[3] |= 3 & 0xF;
		break;
	case 0x40: // S
		Output[3] |= 4 & 0xF;
		break;
	case 0xC0: // SW
		Output[3] |= 5 & 0xF;
		break;
	case 0x80: // W
		Output[3] |= 6 & 0xF;
		break;
	case 0x90: // NW
		Output[3] |= 7 & 0xF;
		break;
	default: // Released
		Output[3] |= 8 & 0xF;
		break;
	}

	// Thumb axes
	Output[4] = Input->LeftThumbX;
	Output[5] = Input->LeftThumbY;
	Output[6] = Input->RightThumbX;
	Output[7] = Input->RightThumbY;

	// Buttons
	Output[0] &= ~0xFF; // Clear all 8 bits
	Output[1] &= ~0xFF; // Clear all 8 bits

	// Face buttons
	Output[0] |= ((Input->Buttons.bButtons[1] & 0xF0) >> 4);
	// L2, R2, L1, R1
	Output[0] |= ((Input->Buttons.bButtons[1] & 0x0F) << 4);

	// Select
	Output[1] |= ((Input->Buttons.bButtons[0] & 0x01) << 1);
	// Start
	Output[1] |= ((Input->Buttons.bButtons[0] & 0x08) >> 3);
	// L3
	Output[1] |= ((Input->Buttons.bButtons[0] & 0x02) << 1);
	// R3
	Output[1] |= ((Input->Buttons.bButtons[0] & 0x04) << 1);
	// PS
	Output[1] |= ((Input->Buttons.bButtons[2] & 0x01) << 4);

	// Trigger axes (inverted)
	Output[10] = (0xFF - Input->Pressure.Values.L2);
	Output[11] = (0xFF - Input->Pressure.Values.R2);

	// Face buttons (pressure, inverted)
	Output[8] = (0xFF - Input->Pressure.Values.Circle);
	Output[9] = (0xFF - Input->Pressure.Values.Cross);
}

UCHAR REVERSE_BITS(UCHAR x)
{
	x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
	x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
	x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
	return x;
}


typedef struct timeval {
	long tv_sec;
	long tv_usec;
} TIMEVAL, * PTIMEVAL, * LPTIMEVAL;

/* FILETIME of Jan 1 1970 00:00:00. */
static const unsigned __int64 epoch = ((unsigned __int64)116444736000000000ULL);

/*
 * timezone information is stored outside the kernel so tzp isn't used anymore.
 *
 * Note: this function is not for Win32 high precision timing purpose. See
 * elapsed_time().
 */
int
gettimeofday(struct timeval* tp, struct timezone* tzp)
{
	FILETIME    file_time;
	SYSTEMTIME  system_time;
	ULARGE_INTEGER ularge;

	GetSystemTime(&system_time);
	SystemTimeToFileTime(&system_time, &file_time);
	ularge.LowPart = file_time.dwLowDateTime;
	ularge.HighPart = file_time.dwHighDateTime;

	tp->tv_sec = (long)((ularge.QuadPart - epoch) / 10000000L);
	tp->tv_usec = (long)(system_time.wMilliseconds * 1000);

	return 0;
}

VOID DS3_RAW_TO_DS4REV1_HID_INPUT_REPORT(
	_In_ PDS3_RAW_INPUT_REPORT Input,
	_Out_ PUCHAR Output,
	_In_ BOOLEAN IsWired, _Out_ PDS3_GYRO_DATA pGyroData, _In_ PVOID pContext
)
{
	PDEVICE_CONTEXT pDevCtx = (PDEVICE_CONTEXT)pContext;

	// Report ID
	Output[0] = Input->ReportId;

	// Prepare D-Pad
	Output[5] &= ~0xF; // Clear lower 4 bits

	// Prepare face buttons
	Output[5] &= ~0xF0; // Clear upper 4 bits

	// Remaining buttons
	Output[6] &= ~0xFF; // Clear all 8 bits

	// PS button
	Output[7] &= ~0x01; // Clear button bit
	Output[7] &= ~0x02; // Clear touchbutton bit
	Output[7] &= ~0xFC; // Clear frameCount bits (upper 6)

	//TimeStamp
	Output[10] &= ~0xFF;
	Output[11] &= ~0xFF;

	//Sixaxis - Gyro - dunno
	Output[13] &= ~0xFF;
	Output[14] &= ~0xFF;
	Output[15] &= ~0xFF;
	Output[16] &= ~0xFF;
	Output[17] &= ~0xFF;
	Output[18] &= ~0xFF;

	//Sixaxis - Acel
	Output[19] &= ~0xFF; //lower 8bits accel x
	Output[20] &= ~0xFF; //uppper 8 bits accel x
	Output[21] &= ~0xFF; // accel y
	Output[22] &= ~0xFF;
	Output[23] &= ~0xFF; // accel z
	Output[24] &= ~0xFF;

	// Battery + cable info
	Output[30] &= ~0xF; // Clear lower 4 bits

	// Finger 1 touchpad contact info
	Output[35] |= 0x80; // Set top bit to disable finger contact
	Output[44] |= 0x80; // Set top bit to disable finger contact

	// Finger 2 touchpad contact info
	Output[39] |= 0x80; // Set top bit to disable finger contact
	Output[48] |= 0x80; // Set top bit to disable finger contact

	Output[33] &= ~0xFF;
	Output[34] &= ~0xFF;

	Output[35] &= ~0x7F; // Clear ID

	Output[36] &= ~0xFF;
	Output[37] &= ~0xFF;
	Output[38] &= ~0xFF;

	Output[39] &= ~0x7F; // Clear ID

	Output[40] &= ~0xFF;
	Output[41] &= ~0xFF;
	Output[42] &= ~0xFF;


	// Translate D-Pad to HAT format
	switch (Input->Buttons.bButtons[0] & ~0xF)
	{
	case 0x10: // N
		Output[5] |= 0 & 0xF;
		break;
	case 0x30: // NE
		Output[5] |= 1 & 0xF;
		break;
	case 0x20: // E
		Output[5] |= 2 & 0xF;
		break;
	case 0x60: // SE
		Output[5] |= 3 & 0xF;
		break;
	case 0x40: // S
		Output[5] |= 4 & 0xF;
		break;
	case 0xC0: // SW
		Output[5] |= 5 & 0xF;
		break;
	case 0x80: // W
		Output[5] |= 6 & 0xF;
		break;
	case 0x90: // NW
		Output[5] |= 7 & 0xF;
		break;
	default: // Released
		Output[5] |= 8 & 0xF;
		break;
	}
	
	// Face buttons
	Output[5] |= ((REVERSE_BITS(Input->Buttons.bButtons[1]) << 4) & 0xF0);
		
	// Select to Share
	Output[6] |= ((Input->Buttons.bButtons[0] & 0x01) << 4);

	// Start to Options
	Output[6] |= (((Input->Buttons.bButtons[0] >> 3) & 0x01) << 5);

	// L1, L2, R1, R2
	Output[6] |= (((Input->Buttons.bButtons[1] >> 2) & 0x01) << 0);
	Output[6] |= (((Input->Buttons.bButtons[1] >> 0) & 0x01) << 2);
	Output[6] |= (((Input->Buttons.bButtons[1] >> 3) & 0x01) << 1);
	Output[6] |= (((Input->Buttons.bButtons[1] >> 1) & 0x01) << 3);

	// L3, R3
	Output[6] |= (((Input->Buttons.bButtons[0] >> 1) & 0x01) << 6);
	Output[6] |= (((Input->Buttons.bButtons[0] >> 2) & 0x01) << 7);
	
	// Thumb axes
	Output[1] = Input->LeftThumbX;
	Output[2] = Input->LeftThumbY;
	Output[3] = Input->RightThumbX;
	Output[4] = Input->RightThumbY;

	// Trigger axes
	Output[8] = Input->Pressure.Values.L2;
	Output[9] = Input->Pressure.Values.R2;

	// PS button
	Output[7] |= Input->Buttons.Individual.PS;

	// Battery translation when IsWired = 0: ( Value * 100 ) / 8
	// Battery translation when IsWired = 1: ( Value * 100 ) / 11
	if (IsWired)
	{
		// Wired sets a flag
		Output[30] |= 0x10;

		switch ((DS_BATTERY_STATUS)Input->BatteryStatus)
		{
		case DsBatteryStatusCharging:
			Output[30] |= 0; // 36%
			break;
		case DsBatteryStatusCharged:
		case DsBatteryStatusFull:
			Output[30] |= 11; // 100%
			break;
		}
	}
	else
	{
		// Clear flag
		Output[30] &= ~0x10;

		switch ((DS_BATTERY_STATUS)Input->BatteryStatus)
		{
		case DsBatteryStatusCharged:
		case DsBatteryStatusFull:
			Output[30] |= 8; // 100%
			break;
		case DsBatteryStatusHigh:
			Output[30] |= 6; // 75%
			break;
		case DsBatteryStatusMedium:
			Output[30] |= 4; // 50%
			break;
		case DsBatteryStatusLow:
			Output[30] |= 2; // 25%
			break;
		case DsBatteryStatusDying:
			Output[30] |= 1; // 12%
			break;
		}
	}

	//Sixaxis translation
	UCHAR alt = 0xFF; //alt read mode;

	//magnitude seems to be a lot less in ds4 land.
	if (alt> 0)
	{

		USHORT ddX = 0x03FF - _byteswap_ushort(Input->AccelerometerX);
		USHORT ddY = _byteswap_ushort(Input->AccelerometerY);
		USHORT ddZ = _byteswap_ushort(Input->AccelerometerZ);
		USHORT dYaw = _byteswap_ushort(Input->Gyroscope); 

		if ((pDevCtx->GyroData.calibModel & 0x4) == 0 &&
			(pDevCtx->GyroData.calibModel & 0x20) == 0 && 
			(pDevCtx->GyroData.calibModel & 0x1) == 0 && 
			(pDevCtx->GyroData.calibModel & 0x2) == 0)
		{
			dYaw = 0x03FF - dYaw;
		}

		// Sensors range 0 - 1023. Zero at 512
        const int a0 = 512;

		//testing sixaxis
		//pGyroData->yawOffset = 0;// (USHORT)a0;
		//int gVal = 1023 - (int)dYaw;

		////clamp
		//if (gVal < 0)
		//	gVal = 0;
		//else if (gVal > 1023)
		//	gVal = 1023;

		//dYaw = (USHORT)gVal;
		int gyroOffset = pGyroData->yawZero == 0 ? a0 : pGyroData->yawZero;

		int AccelX = -(int)((double)((int)ddX - a0)/113.0 * 8192.0);
		int AccelY = -(int)((double)((int)ddY - a0)/113.0 * 8192.0);
		int AccelZ = -(int)((double)((int)ddZ - a0)/113.0 * 8192.0);
		int Gyro = (int)((double)((int)dYaw - gyroOffset ) / 123.0 * 90.0 * 16.0);

		//if (pGyroData->yawOffset == 0){
		//	Gyro = 0;
		//}

		//clamp
		if (AccelX > SHRT_MAX)
			AccelX = SHRT_MAX;
		if (AccelX < SHRT_MIN)
			AccelX = SHRT_MIN;

		if (AccelY > SHRT_MAX)
			AccelY = SHRT_MAX;
		if (AccelY < SHRT_MIN)
			AccelY = SHRT_MIN;

		if (AccelZ > SHRT_MAX)
			AccelZ = SHRT_MAX;
		if (AccelZ < SHRT_MIN)
			AccelZ = SHRT_MIN;

		if (Gyro > SHRT_MAX)
			Gyro = SHRT_MAX;
		if (Gyro < SHRT_MIN)
			Gyro = SHRT_MIN;


		Output[19] |= (UCHAR)((SHORT)AccelX & 0xFF);
		Output[20] |= (UCHAR)((SHORT)AccelX >> 8);

		// 'height' appears to be y-axis on ds4, so swap them around.
		//z-axis is inverted.
		Output[23] |= (UCHAR)((SHORT)AccelY & 0xFF);
		Output[24] |= (UCHAR)((SHORT)AccelY >> 8);

		//y-axis is inverted.
		Output[21] |= (UCHAR)((SHORT)AccelZ & 0xFF);
		Output[22] |= (UCHAR)((SHORT)AccelZ >> 8);

		//yaw
		Output[15] |= (UCHAR)((SHORT)Gyro & 0xFF); // only has 1 sensor for yaw, needs interpolation from accelerometers
		Output[16] |= (UCHAR)((SHORT)Gyro >> 8);

		//pitch and roll angular velocity
		//double roll = atan2(-((int)ddX - a0), -((int)ddZ - a0));

		//https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf, eqn 37, 38
		const double mu = 0.01;
		double azyMu = sqrt(pow((int)ddZ - a0, 2) + mu*pow((int)ddY - a0,2));
		azyMu = -((int)ddZ - a0) > 0 ? azyMu : -azyMu; //sign factor

		double roll = atan2(-((int)ddX - a0), azyMu);

		double azx = hypot((int)ddX - a0, (int)ddZ - a0);
		double pitch = atan2(((int)ddY - a0), azx);

		//struct timeval tv;
		//gettimeofday(&tv, NULL);
		//double now = tv.tv_sec + tv.tv_usec * 1e-6;
		LARGE_INTEGER now;
		LARGE_INTEGER freq;
		QueryPerformanceFrequency(&freq);
		QueryPerformanceCounter(&now);

		//if (!pGyroData->time)
		//{
		//	//pGyroData->startTime = now;
		//	pGyroData->time = now;
		//	pGyroData->pitch = pitch;
		//	pGyroData->roll = roll;
		//}

		if (pGyroData->lastTimeStamp.QuadPart == 0)
        {
			//pGyroData->lastTimeStamp = now;
			pGyroData->lastTimeStamp = now;
            pGyroData->pitch = pitch;
            pGyroData->roll = roll;
        }


		//double dt = now - pGyroData->time;
		double dt = (double)(now.QuadPart - pGyroData->lastTimeStamp.QuadPart) / (double)freq.QuadPart;
		
		double dPitch = dt > 0 ? (pitch - pGyroData->pitch) / dt : 0;
		double deltaRoll = (roll - pGyroData->roll);
		
		//QueryPerformanceCounter(&pGyroData->lastTimeStamp);
		pGyroData->lastTimeStamp = now;
		pGyroData->pitch = pitch;
		pGyroData->roll = roll;

		//pGyroData->timestamp += (dt * 25000);
		pGyroData->timestamp += (USHORT)(dt /*seconds*/ * 1000000 /*mu s*/ * 3u /*typical ms delta*/ / 16u /*typical value delta */);

		//180 -> -180 vice versa
		if (deltaRoll > (1.5 * M_PI))
			deltaRoll -= (2.0 * M_PI);
		if (deltaRoll < -(1.5 * M_PI))
			deltaRoll += (2.0 * M_PI);

		double dRoll = dt > 0 ? deltaRoll / dt : 0;

		//pGyroData->dPitchSum = pGyroData->dPitchSum - pGyroData->dPitch[pGyroData->Index];       // Remove the oldest entry from the sum
		//pGyroData->dRollSum = pGyroData->dRollSum - pGyroData->dRoll[pGyroData->Index];       // Remove the oldest entry from the sum

		//pGyroData->dPitch[pGyroData->Index] = dPitch;           // Add the newest reading to the window
		//pGyroData->dRoll[pGyroData->Index] = dRoll;           // Add the newest reading to the window
		//pGyroData->dPitchSum = pGyroData->dPitchSum + dPitch;                 // Add the newest reading to the sum
		//pGyroData->dRollSum = pGyroData->dRollSum + dRoll;                 // Add the newest reading to the sum

		//pGyroData->Index = (pGyroData->Index + 1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size

		//double avgRoll = pGyroData->dRollSum / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result
		//double avgPitch = pGyroData->dPitchSum / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result

		// Exponential weight de-noise
		// yn = w × xn + (1 – w) × yn – 1
		double const weight = 0.1;
		pGyroData->dRollEst = weight * dRoll + (1.0 - weight) * pGyroData->dRollEst;
		pGyroData->dPitchEst = weight * dPitch + (1.0 - weight) * pGyroData->dPitchEst;
		// moving average de-noise
		//pGyroData->dPitchEst = (pGyroData->dPitchEst * (double)(WINDOW_SIZE - 1) + dPitch) / (double)WINDOW_SIZE;
		//pGyroData->dRollEst = (pGyroData->dRollEst * (double)(WINDOW_SIZE - 1) + dRoll) / (double)WINDOW_SIZE;
		
		//SHORT vRoll = (SHORT)(avgRoll / M_PI * 180 * 16);//SHORT_MAX);
		SHORT vRoll = (SHORT)(pGyroData->dRollEst / M_PI * 180 * 16);//SHORT_MAX);
		SHORT vPitch = (SHORT)(pGyroData->dPitchEst / M_PI * 180 * 16); //SHORT_MAX);

		SHORT sRoll = (SHORT)(roll / M_PI * 180 * 16);
		SHORT sPitch = (SHORT)(pitch / M_PI * 180 * 16);



		//clamp
		if (vPitch > SHRT_MAX)
			vPitch = SHRT_MAX;
		if (vPitch < SHRT_MIN)
			vPitch = SHRT_MIN;

		if (vRoll > SHRT_MAX)
			vRoll = SHRT_MAX;
		if (vRoll < SHRT_MIN)
			vRoll = SHRT_MIN;

		//init yaw when 'pitch' and 'roll' has settled
		if (pGyroData->calibInputDone == FALSE && pGyroData->calibOutputDone == TRUE && abs(vPitch) <= 16 && abs(vRoll) <= 16 && (dt >= 0.000001))
		{
		    pGyroData->yawZero = dYaw;
			pGyroData->calibInputDone = TRUE;
		}
		
		//pitch
		Output[13] |= (UCHAR)(vPitch & 0xFF);
		Output[14] |= (UCHAR)(vPitch >> 8);

		//roll
		Output[17] |= (UCHAR)(vRoll & 0xFF);
		Output[18] |= (UCHAR)(vRoll >> 8);

		//timestamp
		Output[10] |= (UCHAR)((USHORT)pGyroData->timestamp & 0xFF); 
		Output[11] |= (UCHAR)((USHORT)pGyroData->timestamp >> 8);

		//FrameCount
		pGyroData->frameCount = ++pGyroData->frameCount % 64;
		Output[7] |= (UCHAR)(pGyroData->frameCount << 2);
	}

	//TouchPad emulation

	// PS button and trigger pressure detected, enable tp emulation until triggers released
	if ((Input->Buttons.Individual.PS || pGyroData->wasLeftTouch == TRUE || pGyroData->wasRightTouch == TRUE) && (Input->Pressure.Values.L2 > 0 || Input->Pressure.Values.R2 > 0))
	{
		//override ps and triggers
		Output[7] &= ~Input->Buttons.Individual.PS;

		//L2, R2
		Output[6] &= ~(((Input->Buttons.bButtons[1] >> 0) & 0x01) << 2);
		Output[6] &= ~(((Input->Buttons.bButtons[1] >> 1) & 0x01) << 3);

		// Trigger axes
		Output[8] = 0;
		Output[9] = 0;

		// tp click if pressure past a certain point
		if ((Input->Pressure.Values.L2 > 240 || Input->Pressure.Values.R2 > 240))
		    Output[7] |= (0x1 << 1);

		//stick offsets?
		//
		++pGyroData->touchPacketCount;
		Output[33] = 1; //single packet
		Output[34] = pGyroData->touchPacketCount;
		
		//finger 1 data
		if (Input->Pressure.Values.L2 > 0)
		{
			if (pGyroData->wasLeftTouch == FALSE)
			{
				//pGyroData->currentTouchID = ++pGyroData->currentTouchID % 0x7f;
				pGyroData->leftTouchID = ++pGyroData->leftTouchID % 0x80;
			}
			pGyroData->wasLeftTouch = TRUE;
			Output[35] &= ~0x80; // unset to be on.
			Output[35] |= pGyroData->leftTouchID;
			/*
			 *        public const int RESOLUTION_X_MAX = 1920;
        public const int RESOLUTION_Y_MAX = 942;
			 *
			 */
		    // set touch x [36] plus lower [37] y high [37] and [38]

			USHORT xleft = (1920 - 1920 * Input->Pressure.Values.L2 / 255) / 2;
			USHORT yleft = 942 / 2;

			Output[36] |= (UCHAR)(xleft & 0xFF);
			Output[37] |= (UCHAR)((xleft >> 8) & 0x0F);
			Output[37] |= (UCHAR)((yleft << 4) & 0xF0);
			Output[38] |= (UCHAR)((yleft >> 4) & 0xFF);
		}
		else
		{
			pGyroData->wasLeftTouch = FALSE;
		}


		//finger 2 data
		if (Input->Pressure.Values.R2 > 0)
		{
			if (pGyroData->wasRightTouch == FALSE)
			{
				//pGyroData->currentTouchID = ++pGyroData->currentTouchID % 0x80;
				pGyroData->rightTouchID = pGyroData->rightTouchID % 0x80;
			}
			pGyroData->wasRightTouch = TRUE;
			Output[39] &= ~0x80; // unset to be on.
			Output[39] |= pGyroData->rightTouchID;
			/*
 *        public const int RESOLUTION_X_MAX = 1920;
public const int RESOLUTION_Y_MAX = 942;
	 *
	 */
			USHORT xright = 960 + 1920 * Input->Pressure.Values.R2 / 255 / 2;
			USHORT yright = 942 / 2;
			// set touch x [40] plus lower [41] y high [41] and [42]
			Output[40] |= (UCHAR)(xright & 0xFF);
			Output[41] |= (UCHAR)((xright >> 8) & 0x0F);
			Output[41] |= (UCHAR)((yright << 4) & 0xF0);
			Output[42] |= (UCHAR)((yright >> 4) & 0xFF);
		}
		else
		{
			pGyroData->wasRightTouch = FALSE;
		}
	}
	else
	{
		pGyroData->wasLeftTouch = FALSE;
		pGyroData->wasRightTouch = FALSE;
	}
}

VOID DS3_RAW_TO_XINPUTHID_HID_INPUT_REPORT(
	_In_ PDS3_RAW_INPUT_REPORT Input, 
	_Out_ PXINPUT_HID_INPUT_REPORT Output
)
{
	//
	// Thumb axes
	// 
	Output->GD_GamePadX = Input->LeftThumbX * 257;
	Output->GD_GamePadY = Input->LeftThumbY * 257;
	Output->GD_GamePadRx = Input->RightThumbX * 257;
	Output->GD_GamePadRy = Input->RightThumbY * 257;

	//
	// Triggers
	// 
	Output->GD_GamePadZ = Input->Pressure.Values.L2 * 4;
	Output->GD_GamePadRz = Input->Pressure.Values.R2 * 4;

	//
	// Face
	// 
	Output->BTN_GamePadButton1 = Input->Buttons.Individual.Cross;
	Output->BTN_GamePadButton2 = Input->Buttons.Individual.Circle;
	Output->BTN_GamePadButton3 = Input->Buttons.Individual.Square;
	Output->BTN_GamePadButton4 = Input->Buttons.Individual.Triangle;

	//
	// Shoulder
	// 
	Output->BTN_GamePadButton5 = Input->Buttons.Individual.L1;
	Output->BTN_GamePadButton6 = Input->Buttons.Individual.R1;

	//
	// Select & Start
	// 
	Output->BTN_GamePadButton7 = Input->Buttons.Individual.Select;
	Output->BTN_GamePadButton8 = Input->Buttons.Individual.Start;

	//
	// Thumbs
	// 
	Output->BTN_GamePadButton9 = Input->Buttons.Individual.L3;
	Output->BTN_GamePadButton10 = Input->Buttons.Individual.R3;
		
	// 
	// D-Pad (POV/HAT format)
	// 
	switch (Input->Buttons.bButtons[0] & ~0xF)
	{
	case 0x10: // N
		Output->GD_GamePadHatSwitch = 1;
		break;
	case 0x30: // NE
		Output->GD_GamePadHatSwitch = 2;
		break;
	case 0x20: // E
		Output->GD_GamePadHatSwitch = 3;
		break;
	case 0x60: // SE
		Output->GD_GamePadHatSwitch = 4;
		break;
	case 0x40: // S
		Output->GD_GamePadHatSwitch = 5;
		break;
	case 0xC0: // SW
		Output->GD_GamePadHatSwitch = 6;
		break;
	case 0x80: // W
		Output->GD_GamePadHatSwitch = 7;
		break;
	case 0x90: // NW
		Output->GD_GamePadHatSwitch = 8;
		break;
	default: // Released
		Output->GD_GamePadHatSwitch = 0;
		break;
	}

	Output->GD_GamePadSystemControlSystemMainMenu = Input->Buttons.Individual.PS;
}
