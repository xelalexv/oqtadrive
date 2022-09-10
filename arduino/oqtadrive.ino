/*
    OqtaDrive - Sinclair Microdrive emulator
    Copyright (c) 2021, Alexander Vollschwitz

    developed on: Arduino Nano

    This file is part of OqtaDrive.

    OqtaDrive is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    OqtaDrive is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with OqtaDrive. If not, see <http://www.gnu.org/licenses/>.
*/


// ----------------------------------------------------------------- CONFIG ---
//  This section contains all configuration items. Do not change anything below
//  this section unless you know what you're doing!
//
//  If you need to maintain configs for various adapters, you can alternatively
//  place your settings in different header files in the `config` folder next
//  to this file, one per adapter, and `#include` the desired one before
//  uploading. Each header file only needs to contain the settings you want to
//  change. All other settings will remain at their defaults. Note that for
//  convenience, files in the `config` folder are git-ignored.
//
//#include "config/dongle.h"
//#include "config/spectrum.h"
//#include "config/if1.h"
//#include "config/ql.h"
//#include "config/pi.h"

//  Set whether read & write LEDs should be on when idling.
#ifndef LED_RW_IDLE_ON
#define LED_RW_IDLE_ON true
#endif

// Set whether the read & write LEDs should indicate that the adapter is
// waiting to sync with the daemon (LEDs alternate)
#ifndef LED_SYNC_WAIT
#define LED_SYNC_WAIT true
#endif

// rumble strength; this is a PWM setting (0-255), set to 0 for off
#ifndef RUMBLE_LEVEL
#define RUMBLE_LEVEL 35
#endif

/*
    Automatic offset check only works for QL. If you're using OqtaDrive with an
    actual Microdrive between IF1 and the adapter, you can set a fixed offset
    here. Likewise for the QL, if the automatic check doesn't work reliably.
    The offset denotes how many actual Microdrives are present between the
    Microdrive interface and the adapter. So an offset of 0 means the adapter is
    directly connected to the IF1 or internal Microdrive interface on the QL,
    bypassing the two built-in drives. Max accepted value is 7. Keep at -1 to
    use automatic offset check.
 */
#ifndef DRIVE_OFFSET_IF1
#define DRIVE_OFFSET_IF1 0
#endif

#ifndef DRIVE_OFFSET_QL
#define DRIVE_OFFSET_QL -1
#endif

/*
    If you want to map hardware drives, i.e. move them to different slots within
    the daisy chain, you need to chain them behind the OqtaDrive adapter. This
    requires routing the COMMS_OUT signal from the adapter to the COMMS_IN of
    the first hardware drive. See the documentation for more details.

    Once you have set up OqtaDrive in this way, you can define here to which
    slots the drives are mapped after the adapter starts up. During operation
    you can then control the mapping via oqtactl. The hardware drives are always
    mapped as a group. Setting start and end to 0 will deactivate the hardware
    drives. Setting HW_GROUP_LOCK to true will lock the group settings so that
    they cannot be changed with oqtactl.

    Note: Set offsets above to 0, since hardware drive mapping requires the
          OqtaDrive adapter to be first in the chain.
 */
#ifndef HW_GROUP_START
#define HW_GROUP_START 0
#endif

#ifndef HW_GROUP_END
#define HW_GROUP_END 0
#endif

#ifndef HW_GROUP_LOCK
#define HW_GROUP_LOCK true
#endif

//  Use these settings to force either Interface 1 or QL, but not both! When
//  left at false, automatic detection is used.
#ifndef FORCE_IF1
#define FORCE_IF1 false
#endif

#ifndef FORCE_QL
#define FORCE_QL false
#endif

/*
    This is the baud rate of the serial link between adapter and daemon. It is
    highly recommended to use the value defined here, which is the minimum speed
    required for error free communication, and at the same time the maximum speed
    at which an Arduino Nano can reliably operate the serial port. On some boards
    used for running the daemon, such as the BananaPi M2 Zero, the 1 Mbps speed
    is not available due to the combination of frequency and divider used to
    clock its UART. For this platform, 500 kbps is currently being tested and
    may work.

 */
#ifndef BAUD_RATE
#define BAUD_RATE 1000000
#endif

/*
	If write protect is not working reliably with your Spectrum, the voltage
	asserted to the /WR.PR line may not be high enough. An original Microdrive
	outputs 9V to signal a writable cartridge, while OqtaDrive only outputs 5V.
	In most cases, this is enough, but there have been reports about problems
	with this. In this case, insert a transistor into the WR.PR output as
	described in the project README, and change this setting to true.
 */
#ifndef WR_PROTECT_BOOST
#define WR_PROTECT_BOOST false
#endif

// ----------------------------------- END OF CONFIG - START OF DANGER ZONE ---

#include <EEPROM.h>

/*
	Implementation notes:
	- _delay_us only takes compile time constants as argument
 */

#define FIRMWARE_VERSION 24

// Change this to true for a calibration run. When not connecting the adapter to
// an Interface 1/QL during calibration, choose the desired interface via the
// force settings below.
#define CALIBRATION false

// --- pin assignments --------------------------------------------------------
//
// Note: Changing pin assignments (other than LED pins) will break things!
//       These constants are for clarity & convenience only.
//
const int PIN_COMMS_CLK  = 2; // HIGH idle on IF1, LOW on QL; interrupt
const int PIN_COMMS_IN   = 4;
const int PIN_COMMS_OUT  = 7;
const int PIN_ERASE      = 5; // LOW active
const int PIN_READ_WRITE = 3; // READ is HIGH; interrupt
const int PIN_WR_PROTECT = 6; // LOW active

const int PIN_RUMBLE     = 10;
const int PIN_LED_WRITE  = 11;
const int PIN_LED_READ   = 12;

const int PIN_TRACK_1 = A4;
const int PIN_TRACK_2 = A0;

// --- pin masks --------------------------------------------------------------
const uint8_t MASK_COMMS_CLK  = 1 << PIN_COMMS_CLK;
const uint8_t MASK_COMMS_IN   = 1 << PIN_COMMS_IN;
const uint8_t MASK_COMMS_OUT  = 1 << PIN_COMMS_OUT;
const uint8_t MASK_ERASE      = 1 << PIN_ERASE;
const uint8_t MASK_READ_WRITE = 1 << PIN_READ_WRITE;
const uint8_t MASK_WR_PROTECT = 1 << PIN_WR_PROTECT;
const uint8_t MASK_RECORDING  = MASK_ERASE | MASK_READ_WRITE;

const uint8_t MASK_TRACK_1     = B00010000;
const uint8_t MASK_TRACK_2     = B00000001;
const uint8_t MASK_BOTH_TRACKS = MASK_TRACK_1 | MASK_TRACK_2;

const uint8_t MASK_LED_WRITE  = B00001000;
const uint8_t MASK_LED_READ   = B00010000;

// --- LED behavior & rumble --------------------------------------------------
const bool ACTIVE = true;
const bool IDLE   = false;

uint8_t blinkCount  = 0;
uint8_t rumbleLevel = RUMBLE_LEVEL;

// --- tape format ------------------------------------------------------------
const int PREAMBLE_LENGTH   = 12;
const int HEADER_LENGTH_IF1 = 27;
const int RECORD_LENGTH_IF1 = 540;
const int HEADER_LENGTH_QL  = 28;
const int RECORD_LENGTH_QL  = 538;
const int RECORD_EXTRA_IF1  = 99; // during format, Spectrums with earlier ROMs
const int RECORD_EXTRA_QL   = 86; // and QLs send longer records

uint16_t headerLengthMux;
uint16_t recordLengthMux;
uint16_t sectorLengthMux;

// --- timer pre-loads --------------------------------------------------------
// values lower than 256 use a 256 pre-scaler (1 tick is 16us), values 256 and
// above use a 1024 pre-scaler (1 tick is 64us)
const int TIMER_COMMS          = 512 - 157; //   10 msec
const int TIMER_HEADER_GAP_IF1 = 256 - 234; // 3.75 msec
const int TIMER_HEADER_GAP_QL  = 256 - 225; // 3.60 msec

// --- drive select -----------------------------------------------------------
volatile uint8_t commsRegister   = 0;
volatile uint8_t commsClkCount   = 0;
volatile uint8_t activeDrive     = 0;
volatile uint8_t lastActiveDrive = 0;
volatile uint8_t driveOffset     = 0xff;
volatile uint8_t hwGroupStart    = 0;
volatile uint8_t hwGroupEnd      = 0;
volatile uint8_t maskHwOffset    = 0;

bool commsClkState;

const uint8_t DRIVE_STATE_UNKNOWN  = 0x80;
const uint8_t DRIVE_FLAG_LOADED    = 1;
const uint8_t DRIVE_FLAG_FORMATTED = 2;
const uint8_t DRIVE_FLAG_READONLY  = 4;
const uint8_t DRIVE_READABLE = DRIVE_FLAG_LOADED | DRIVE_FLAG_FORMATTED;

volatile uint8_t driveState = DRIVE_STATE_UNKNOWN;

// --- interrupt handler ------------------------------------------------------
typedef void (* TimerHandler)();
void setTimer(int preload, TimerHandler h);
void enableTimer(bool on, int preload, TimerHandler h);
TimerHandler timerHandler = NULL;

// --- sector buffer ----------------------------------------------------------
const uint16_t BUF_LENGTH = 10 +
	max(HEADER_LENGTH_IF1, HEADER_LENGTH_QL) +
	max(RECORD_LENGTH_IF1 + RECORD_EXTRA_IF1, RECORD_LENGTH_QL + RECORD_EXTRA_QL);
uint8_t buffer[BUF_LENGTH];

// --- message buffer ---------------------------------------------------------
uint8_t msgBuffer[4];

// --- Microdrive interface type - Interface 1 or QL --------------------------
bool IF1 = true;
#define QL !IF1

// --- state flags ------------------------------------------------------------
volatile bool spinning    = false;
volatile bool recording   = false;
volatile bool message     = false;
volatile bool headerGap   = false;
volatile bool calibration = false; // use the define setting at top to turn on!
volatile bool synced      = false;

// --- daemon commands --------------------------------------------------------
const uint8_t PROTOCOL_VERSION = 4;

const char CMD_HELLO   = 'h';
const char CMD_VERSION = 'v';
const char CMD_PING    = 'P';
const char CMD_STATUS  = 's';
const char CMD_GET     = 'g';
const char CMD_PUT     = 'p';
const char CMD_VERIFY  = 'y';
const char CMD_MAP     = 'm';
const char CMD_DEBUG   = 'd';
const char CMD_RESYNC  = 'r';
const char CMD_CONFIG  = 'c';

const char CMD_CONFIG_RUMBLE  = 'r';

const uint8_t  CMD_LENGTH = 4;
const uint16_t PAYLOAD_LENGTH = BUF_LENGTH - CMD_LENGTH;

const char DAEMON_PING[]  = {CMD_PING, 'i', 'n', 'g'};
const char DAEMON_PONG[]  = {CMD_PING, 'o', 'n', 'g'};
const char DAEMON_HELLO[] = {CMD_HELLO, 'l' , 'o', 'd'};
const char IF1_HELLO[]    = {CMD_HELLO, 'l' , 'o', 'i'};
const char QL_HELLO[]     = {CMD_HELLO, 'l' , 'o', 'q'};

const uint8_t MASK_IF1 = 1;
const uint8_t MASK_QL  = 2;

const unsigned long DAEMON_TIMEOUT   =  5000;
const unsigned long RESYNC_THRESHOLD =  4500;
const unsigned long PING_INTERVAL    = 10000;

unsigned long lastPing = 0;

// ------------------------------------------------------------------ SETUP ---

//
void setup() {

	deactivateSignals();

	loadState();

	// FIXME: does this need deactivation?
	pinMode(PIN_COMMS_OUT, OUTPUT);
	digitalWrite(PIN_COMMS_OUT, LOW);

	// rumble & LEDs
	pinMode(PIN_RUMBLE, OUTPUT);
	pinMode(PIN_LED_WRITE, OUTPUT);
	pinMode(PIN_LED_READ, OUTPUT);
	ledRead(IDLE);
	ledWrite(IDLE);

	// open channel to daemon & say hello
	detectInterface(false, false);
	Serial.begin(BAUD_RATE, SERIAL_8N1);
	Serial.setTimeout(DAEMON_TIMEOUT);

	// set up interrupts
	attachInterrupt(digitalPinToInterrupt(PIN_COMMS_CLK), commsClk, CHANGE);
	attachInterrupt(digitalPinToInterrupt(PIN_READ_WRITE), writeReq, FALLING);

	rumbleGreet();
	startupTests();
}

//
void startupTests() {
	if (!testBuffer()) {
		errorBlink(1, 2, 3); // bad RAM
	}
}

//
void remoteConfig(uint8_t item, uint8_t arg1, uint8_t arg2) {
	switch (item) {
		case CMD_CONFIG_RUMBLE:
			rumbleLevel = arg1;
			rumbleGreet();
			saveState();
			break;
	}
}

//
void setHWGroup(uint8_t start, uint8_t end) {
	if (start <= 8 && end <= 8 && start <= end) {
		hwGroupStart = start;
		hwGroupEnd = end;
		maskHwOffset = 1 << (start - 2);
	}
}

// ------------------------------------------------------------------- LOOP ---

void loop() {

	// FIXME: verify that serial is only accessed from main loop

	if (CALIBRATION) {
		calibrate(0x0f);
	}

	if (!synced) {
		driveOff();
		daemonSync();
	}

	debugFlush();
	ensureDriveState();

	if (spinning) {

		if (recording) {
			record();
		} else {
			replay();
		}

		if (blinkCount++ % 2 == 0) {
			if (recording) {
				ledRead(IDLE);
				ledWriteFlip();
			} else {
				ledWrite(IDLE);
				ledReadFlip();
			}
		}

		lastPing = millis();

	} else if (daemonPing()) {
		// After a successful ping exchange, open a brief window during which
		// the daemon may send control commands. The daemon must not send
		// commands at any other time.
		daemonCheckControl();
	}
}

// ---------------------------------------------- Interface 1 / QL HANDLING ---

void detectInterface(bool if1, bool ql) {

	if1 = FORCE_IF1 ? true : if1;
	ql = FORCE_QL ? true : ql;

	if (if1) {
		IF1 = true;

	} else if (ql) {
		IF1 = false;

	} else {
		// Idle level of COMMS_CLK is HIGH for Interface 1, LOW for QL.
		// Sample COMMS_CLK line for two seconds to find out.

		// avoid floating input during detect
		pinMode(PIN_COMMS_CLK, INPUT_PULLUP);

		uint8_t high = 0, low = 0;
		for (uint8_t i = 0; i < 21; i++) {
			(PIND & MASK_COMMS_CLK) == 0 ? low++ : high++;
			delay(100);
		}
		IF1 = high > low;

		// restore to input without pull-up; whenever detecInterface is called,
		// COMMS_CLK input is configured that way; should that ever change, we
		// need to get current state above and then restore here
		pinMode(PIN_COMMS_CLK, INPUT);
	}

	if (IF1) {
		if ((-1 < DRIVE_OFFSET_IF1) && (DRIVE_OFFSET_IF1 < 8)) {
			driveOffset = DRIVE_OFFSET_IF1;
		}
		headerLengthMux = HEADER_LENGTH_IF1 + 1;
		recordLengthMux = RECORD_LENGTH_IF1 + 1;
		commsClkState = true;

	} else {
		if ((-1 < DRIVE_OFFSET_QL) && (DRIVE_OFFSET_QL < 8)) {
			driveOffset = DRIVE_OFFSET_QL;
		}
		headerLengthMux = HEADER_LENGTH_QL + 1;
		recordLengthMux = RECORD_LENGTH_QL + 1;
		commsClkState = false;
	}

	sectorLengthMux = headerLengthMux + recordLengthMux;
}

// ----------------------------------------------------- READ/WRITE CONTROL ---

/*
	Check whether the WRITE or ERASE line is active (indicates recording),
	or an impending drive state change is indicated. Switches recording and
	spinning states accordingly, and returns true if any of the above is the
	case. This is used repeatedly while in replay mode to find out whether we
	need to bail out.

	Note: Any change in this function requires re-calibration of replay!

	TODO: Consider whether to consolidate this with checkReplayOrStop, since
	      most of the code is the same. We may not want to do this though for
	      timing reasons.
 */
bool checkRecordingOrStop() {

	uint8_t state = PIND;

	// turn recording on, but never off here
	recording = recording || ((state & MASK_RECORDING) != MASK_RECORDING);

	// when recording, flip track mode here already
	// to give it more time to switch over
	if (recording) {
		setTracksToRecord();
	}

	// turn spinning off, but never on here; change in drive state is indicated
	// for IF1 by COMMS_CLK going active (i.e. LOW), and for QL by going inactive
	// (also LOW)
	bool prev = commsClkState;
	commsClkState = (state & MASK_COMMS_CLK) != 0;
	spinning = spinning && (!prev || commsClkState);

	return !spinning || recording;
}

/*
	Check whether both WRITE and ERASE lines are inactive (indicates replay),
	or an impending drive state change is indicated. Switches recording and
	spinning states accordingly, and returns true if any of the above is the
	case. This is used repeatedly while in record mode to find out whether we
	need to bail out.
 */
bool checkReplayOrStop() {

	uint8_t state = PIND;

	// turn recording off, but never on here
	recording = recording && ((state & MASK_RECORDING) != MASK_RECORDING);

	if (!recording) {
		setTracksToReplay();
	}

	// turn spinning off, but never on here; change in drive state is indicated
	// for IF1 by COMMS_CLK going active (i.e. LOW), and for QL by going inactive
	// (also LOW)
	bool prev = commsClkState;
	commsClkState = (state & MASK_COMMS_CLK) != 0;
	spinning = spinning && (!prev || commsClkState);

	return !(spinning && recording);
}

// interrupt handler; switches tracks to record mode
void writeReq() {
	if (activeDrive > 0) {
		setTracksToRecord();
	}
}

// interrupt handler; signals the end of the header gap
void endHeaderGap() {
	headerGap = false;
}

/*
	Tracks in RECORD mode means the Arduino reads incoming data. I.e. when the
	Interface 1/QL wants to write to the Microdrive, the two track data pins
	need to be put into input mode.
 */
void setTracksToRecord() {
	DDRC = 0;
	PORTC = 0x3f;
}

/*
	Tracks in REPLAY mode means the Arduino sends data. I.e. when the
	Interface 1/QL wants to read from the Microdrive, the two track data pins
	need to be put into	output mode.
 */
void setTracksToReplay() {
	DDRC = MASK_BOTH_TRACKS;
	PORTC = 0x3f; // idle level is HIGH
}

//
void setWriteProtect(bool protect) {

	if (WR_PROTECT_BOOST) {
		// This is used when an additional PNP transistor is added to pin D6,
		// to overcome problems with write protect always being on (see README).
		if (protect) { // think inverted here
			// To signal that the cartridge is write protected, we must not
			// assert any voltage on the WR.PR line, so we need to keep the
			// transistor off. This is done by switching the pin to input
			// mode, i.e. no voltage, high impedance.
			pinMode(PIN_WR_PROTECT, INPUT);
		} else {
			// To signal that the cartridge is writable, we need to assert 9V
			// on WR.PR, so we output LOW on pin D6, to drive the transistor
			// open.
			pinMode(PIN_WR_PROTECT, OUTPUT);
			digitalWrite(PIN_WR_PROTECT, LOW);
		}

	} else {
		// no transistor, directly driving WR.PR with pin D6
		pinMode(PIN_WR_PROTECT, OUTPUT);
		digitalWrite(PIN_WR_PROTECT, protect ? LOW : HIGH);
	}
}

//
void activateSignals() {
	// control signals
	pinMode(PIN_READ_WRITE, INPUT_PULLUP);
	pinMode(PIN_ERASE, INPUT_PULLUP);
	pinMode(PIN_COMMS_CLK, INPUT_PULLUP);

	// This must not be set to INPUT_PULLUP. If there is a Microdrive upstream
	// of the adapter in the daisy chain, the pull-up resistor would feed into
	// that drive's COMMS_CLK output and confuse it.
	pinMode(PIN_COMMS_IN, INPUT);
}

// When drive is off, signal lines should be set to the least intrusive mode
// possible, to avoid interfering with any actual Microdrives that may be
// present. This is INPUT without pull-up, which for the ATmega328P has a
// typical input leakage current of 1uA.
void deactivateSignals() {
	// control signals
	pinMode(PIN_READ_WRITE, INPUT);
	pinMode(PIN_ERASE, INPUT);
	pinMode(PIN_COMMS_CLK, INPUT);
	pinMode(PIN_COMMS_IN, INPUT);
	pinMode(PIN_WR_PROTECT, INPUT);

	// tracks off
	DDRC = 0;
	PORTC = 0;
}

// ---------------------------------------------------------- DRIVE CONTROL ---

//
void driveOff() {
	stopTimer();
	deactivateSignals();
	ledWrite(IDLE);
	ledRead(IDLE);
	spinning = false;
	recording = false;
	headerGap = false;
	rumble(false);
}

//
void driveOn() {
	activateSignals();
	setTracksToReplay();
	headerGap = false;
	driveState = DRIVE_STATE_UNKNOWN;
	recording = false;
	spinning = true;
	rumble(true);
}

// Active level of COMMS_CLK is LOW for Interface 1, HIGH for QL.
bool isCommsClk(uint8_t state) {
	return (state & MASK_COMMS_CLK) == (IF1 ? 0 : MASK_COMMS_CLK);
}

/*
	Interrupt handler for handling the 1 bit being pushed through the shift
	register formed by the up to eight actual Microdrives (daisy chain), to
	select the active drive. If the OqtaDrive adapter is connected directly to
	the Microdrive interface, commsRegister represents the complete shift
	register. If there are actual Microdrives present before the adapter, then
	commsRegister only represents the remainder of that shift register.
 */
void commsClk() {

	uint8_t d = PIND;
	bool comms = (d & MASK_COMMS_IN) != 0;

	if (hwGroupStart == 1) {
		// immediately pass through COMMS when h/w drives are first in chain
		PORTD = comms ? d | MASK_COMMS_OUT : d & ~MASK_COMMS_OUT;
	}

	if (isCommsClk(d)) {
		// When COMMS_CLK goes active, we increase the clock count, shift the
		// register by one, and add current COMMS state at the start.

		stopTimer();

		if ((driveOffset == 0xff) && QL && comms && (commsClkCount < 8)) {
			// When we see the 1 bit at COMMS_CLK going active, clock count is
			// the drive offset, with an offset of 0 meaning first drive in chain.
			driveOffset = commsClkCount;
		}

		commsClkCount++;
		commsRegister = commsRegister << 1;
		if (comms) {
			commsRegister |= 1;
		}

	} else {
		// COMMS_CLK going inactive triggers the shift register. If there are
		// h/w drives present further down the chain, we therefore sent them
		// the COMMS signal here, with the correct delay using commsRegister.
		if (hwGroupStart > 1) {
			PORTD = (commsRegister & maskHwOffset) != 0 ?
				d | MASK_COMMS_OUT : d & ~MASK_COMMS_OUT;
		}
		setTimer(TIMER_COMMS, selectDrive);
	}
}

/*
	Called by timer when TIMER_COMMS expires. That is, if for a duration of
	TIMER_COMMS, there has been no change on the COMMS_CLK line, then the active
	drive has been selected, or all drives deselected, by Interface 1/QL.
 */
void selectDrive() {

	if (QL && isCommsClk(PIND)) {
		// QL switches COMMS_CLK to HIGH and keeps it HIGH as long as its
		// interested in reading more data. Should the timer fire when we're
		// still reading, we just discard it.
		return;
	}

	if (!synced) {
		return;
	}

	uint8_t last = activeDrive;
	activeDrive = 0;

	// A drive offset of 0xff means we don't know yet, and that also means
	// none of the virtual drives can possibly have been selected.
	if (driveOffset != 0xff) {
		for (uint8_t reg = IF1 ? commsRegister : commsRegister << driveOffset;
			reg > 0; reg >>= 1) {
			activeDrive++;
		}
	}

	debugMsg('C', 'K', commsClkCount);
	debugFlush();
	debugMsg('C', 'R', commsRegister);
	debugFlush();
	debugMsg('O', 'F', driveOffset);
	debugFlush();
	debugMsg('D', 'R', activeDrive);
	debugFlush();

	if (driveOffset != 0xff || commsClkCount > 7) {
		// As soon as the drive offset has been determined, we can reset comms
		// clock count each time a drive is selected. We can do this also if the
		// count goes beyond the maximum offset (7), which happens when
		// resetting the QL.
		commsClkCount = 0;
	}

	// avoid turning on a virtual drive when a h/w drive has been selected
	if (activeDrive <= driveOffset
		|| (hwGroupStart <= activeDrive && activeDrive <= hwGroupEnd)) {
		activeDrive = 0;
	}

	if (activeDrive == 0) {
		lastActiveDrive = last;
		driveOff();
	} else {
		lastActiveDrive = activeDrive;
		driveOn();
	}
}

/*
	Retrieve the drive state from the daemon, if it is still unknown.
	Otherwise, cached value is used.
 */
void ensureDriveState() {

	if (spinning && (driveState != DRIVE_STATE_UNKNOWN)) {
		return;
	}

	if (!spinning && (driveState == DRIVE_STATE_UNKNOWN)) {
		return;
	}

	// When a drive is turned off, selectDrive() may have been hit earlier then
	// this function (seen with QL + Minerva ROM), so activeDrive will already
	// be 0. Previous active drive is therefore saved in lastActiveDrive.
	uint8_t drive = spinning ? activeDrive :
		(activeDrive == 0 ? lastActiveDrive : activeDrive);

	daemonCmdArgs(CMD_STATUS, drive, spinning ? 1 : 0, 0, 0);

	if (spinning) {
		for (int r = 0; r < 400; r++) {
			if (Serial.available() > 0) {
				driveState = Serial.read();
				setWriteProtect(!isDriveWritable());
				return;
			}
			delay(5);
		}
		synced = false;

	} else {
		driveState = DRIVE_STATE_UNKNOWN;
	}
}

//
bool isDriveReadable() {
	return (driveState & DRIVE_READABLE) == DRIVE_READABLE;
}

//
bool isDriveWritable() {
	return (driveState & DRIVE_FLAG_READONLY) == 0;
}

// -------------------------------------------------------------- RECORDING ---

void record() {

	bool formatting = false;
	uint8_t blocks = 0;
	ledWrite(ACTIVE);

	do {
		daemonPendingCmd(CMD_PUT, activeDrive, 0);
		uint16_t read = receiveBlock();
		blocks++;

		if (blocks % 4 == 0) {
			ledWriteFlip();
		}

		// block stop marker; used by the daemon to get rid of spurious
		// extra bytes at the end of a block, often seen on the QL
		if (read > 0) {
			daemonCmdArgs(3, 2, 1, 0, 0);
		}

		read += PREAMBLE_LENGTH; // preamble is not sent to daemon

		if (read < headerLengthMux) {
			break; // nothing useful received
		} else if (read < recordLengthMux) {
			// headers are only written during format
			// when that happens, we stay here
			formatting = true;
			ledRead(IDLE);
		}

	} while (formatting);

	if (formatting) {
		driveState = DRIVE_FLAG_LOADED | DRIVE_FLAG_FORMATTED;
	}

	ledWrite(IDLE);
	checkReplayOrStop();
}

/*
	Receive a block (header or record). Change in drive state is checked only
	while waiting for the start of the block. Once the block starts, no further
	checks are done. End of data from Interface 1/QL however is detected.

	We're reading both tracks simultaneously. Pin assignments are chosen such
	that one track is at bit position 0, the other at 4. 4 bits from each track
	are ORed into `d`, with a left shift before each OR.

                          --------------- track 1
                          |           --- track 2
           bit            |           |
      position:  7  6  5  4  3  2  1  0
          PINC:  X  X  X  |  X  X  X  |   X = don't care
                   A N D  |           |
          MASK:  0  0  0  1  0  0  0  1
                    O R   |           |
             d: [ track 1 *][ track 2 *]  << before each OR

	`d` is then forwarded to the daemon over the serial line. This is repeated
	until the block (header or record) is done. The number of bytes read is
	returned. The receiving side takes care of demuxing the data, additionally
	considering that the tracks are shifted by 4 bits relative to one another.
 */
uint16_t receiveBlock() {

	noInterrupts();

	register uint8_t start = PINC & MASK_BOTH_TRACKS, end, bitCount, d, w;
	register uint16_t read = 0, ww;

	for (ww = 0xffff; ww > 0; ww--) {
		// sync on first bit change of block, on either track
		if (((PINC & MASK_BOTH_TRACKS) ^ start) != 0) {
			break;
		}
		// but don't wait forever, and bail out when activity on COMMS_CLK
		// indicates impending change of drive state
		if (checkReplayOrStop() || ww == 1) {
			UDR0 = ww == 1 ? 2 : 1; // cancel pending PUT command
			interrupts();
			return 0;
		}
	}

	// search for sync pattern, which is 10*'0' followed by 2*'ff'; we therefore
	// look for at least 24 consecutive zeros on both tracks, followed by eight
	// ones on at least one track.
	register uint8_t zeros = 0, ones = 0;
	while (zeros < 24 || ones < 8) {

		for (w = 0xff; (((PINC & MASK_BOTH_TRACKS) ^ start) == 0) && w > 0; w--);

		if (w == 0) { // could not sync
			UDR0 = 3; // cancel pending PUT command
			interrupts();
			return 0;
		}

		_delay_us(2.0); // short delay to make sure track state has settled
		start = PINC & MASK_BOTH_TRACKS;

		if (IF1) _delay_us(6.50); else _delay_us(4.50);

		if (((end = PINC & MASK_BOTH_TRACKS) ^ start) == 0) {
			if (ones > 0) {
				ones = 0;
				zeros = 1;
			} else {
				zeros++;
			}
		} else {
			ones++;
		}
		start = end;
	}

	UDR0 = 0; // complete pending PUT command to go ahead

	while (true) {

		d = 0;

		for (bitCount = 4; bitCount > 0; bitCount--) {

			// wait for start of cycle, or end of block
			// skipped when coming here for the first time
			for (w = 0x2f; (((PINC & MASK_BOTH_TRACKS) ^ start) == 0) && w > 0; w--);

			if (w == 0) { // end of block
				interrupts();
				return read;
			}

			_delay_us(2.0); // short delay to make sure track state has settled
			start = PINC & MASK_BOTH_TRACKS;        //   then take start reading
			                                        // and wait for end of cycle
			if (IF1) _delay_us(6.50); else _delay_us(4.50);
			// When a track has changed state compared to start of cycle at this
			// point, then it carries a 1 in this cycle, otherwise a 0.
			d = (d << 1) | ((end = PINC & MASK_BOTH_TRACKS) ^ start);   // store
			start = end;                               // prepare for next cycle
		}

		UDR0 = d; // send over serial
		read++;
	}
}

// ----------------------------------------------------------------- REPLAY ---

/*
	Replay one sector. Switching over into record mode or stopping of drive is
	continuously monitored, and if detected, returns immediately.
 */
void replay() {

	if (checkRecordingOrStop() || !isDriveReadable()) {
		return;
	}

	daemonCmdArgs(CMD_GET, activeDrive, 0, 0, 0);

	unsigned long start = millis();
	uint16_t rcv = daemonRcv(0);
	if (rcv == 0) {
		synced = millis() - start < RESYNC_THRESHOLD;
		return;
	}

	// header
	if (replayBlock(buffer + CMD_LENGTH, headerLengthMux)) {
		return;
	}

	headerGap = true;
	if (timerEnabled()) { // COMMS_CLK timer may be active at this point
		stopTimer();
	}
	setTimer(IF1 ? TIMER_HEADER_GAP_IF1 : TIMER_HEADER_GAP_QL, endHeaderGap);

	while (headerGap) {
		if (checkRecordingOrStop()) {
			return;
		}
		_delay_us(20.0);
	}

	// record - for QL, this can be two different lengths due to extra bytes
	// being sent during format
	replayBlock(buffer + CMD_LENGTH + headerLengthMux, rcv - headerLengthMux);
}

/*
	Get a sector from daemon and immediately reflect it back. For reliability
	testing. FIXME: validate
 */
void verify() {
	daemonCmdArgs(CMD_GET,
		lowByte(sectorLengthMux), highByte(sectorLengthMux), ' ', 0);
	daemonRcv(sectorLengthMux);
	// send back for verification
	daemonCmdArgs(CMD_VERIFY,
		lowByte(sectorLengthMux), highByte(sectorLengthMux), ' ',
		sectorLengthMux);
}

/*
	Indefinitely replays the given pattern for checking wave form with
	oscilloscope.
 */
void calibrate(uint8_t pattern) {
	calibration = true;
	for (int ix = 0; ix < BUF_LENGTH; ix++) {
		buffer[ix] = pattern;
	}
	setTracksToReplay();
	while (true) {
		replayBlock(buffer, BUF_LENGTH);
	}
}

/*
	Replay a block of bytes to the Interface 1/QL. Switching over to recording
	and stopping of drive is periodically checked. If either of those was
	detected, replay stops and true is returned. If replay was completed,
	false is returned.
 */
bool replayBlock(uint8_t* buf, uint16_t len) {

	noInterrupts();

	register uint8_t bitCount, d, tracks = MASK_BOTH_TRACKS;

	for (; len > 0; len--) {
		d = *buf;
		for (bitCount = 4; bitCount > 0; bitCount--) {
			tracks = ~tracks;            // tracks always flip at start of cycle
			PORTC = tracks | ~MASK_BOTH_TRACKS;                     // cycle end
			                                         // wait for middle of cycle
			if (IF1) _delay_us(5.40); else _delay_us(4.20);
			tracks = tracks ^ d;                   // flip track where data is 1
			PORTC = tracks | ~MASK_BOTH_TRACKS;         // write out track flips
			                                            // wait for end of cycle
			if (IF1) _delay_us(2.35); else _delay_us(1.15);
			// note that calibration must not be a compile time constant,
			// otherwise we'd get different timing due to optimizations
			if (checkRecordingOrStop() && !calibration) {
				interrupts();
				return true;
			}
			d = d >> 1;
		}
		buf++;
	}

	PORTC = 0x3f; // return tracks to idle level (HIGH) at cycle end

	interrupts();
	return false;
}

// --------------------------------------------------------- DEBUG MESSAGES ---

void debugMsg(uint8_t a, uint8_t b, uint8_t c) {
	msgBuffer[1] = a;
	msgBuffer[2] = b;
	msgBuffer[3] = c;
	message = true;
}

void debugFlush() {
	if (message) {
		msgBuffer[0] = CMD_DEBUG;
		Serial.write(msgBuffer, 4);
		Serial.flush();
		message = false;
	}
}

// --------------------------------------------------- DAEMON COMMUNICATION ---

//
void daemonSync() {

	driveState = DRIVE_STATE_UNKNOWN;

	if (LED_SYNC_WAIT) {
		ledRead(true);
		ledWrite(false);
	}

	while (true) {

		daemonCmd((uint8_t*)(IF1 ? IF1_HELLO : QL_HELLO));

		if (daemonRcvAck(10, 100, (uint8_t*)DAEMON_HELLO)) {
			// send protocol & firmware version
			daemonCmdArgs(CMD_VERSION, PROTOCOL_VERSION, FIRMWARE_VERSION, 1, 0);
			// send config
			daemonCmdArgs(CMD_CONFIG, CMD_CONFIG_RUMBLE, rumbleLevel, 0, 0);
			// send h/w drive setup
			daemonHWGroup();
			lastPing = millis();
			synced = true;

			if (LED_SYNC_WAIT) {
				ledRead(IDLE);
				ledWrite(IDLE);
			}
			return;
		}

		if (LED_SYNC_WAIT) {
			ledReadFlip();
			ledWriteFlip();
		}
	}
}

//
void daemonCheckControl() {

	uint8_t arg1, arg2, arg3;

	while (daemonRcvCmd(5, 2)) {

		arg1 = buffer[CMD_LENGTH + 1];
		arg2 = buffer[CMD_LENGTH + 2];
		arg3 = buffer[CMD_LENGTH + 3];

		switch (buffer[CMD_LENGTH]) {

			case CMD_MAP:
				if (!HW_GROUP_LOCK) {
					setHWGroup(arg1, arg2);
					saveState();
				}
				daemonHWGroup();
				break;

			case CMD_CONFIG:
				remoteConfig(arg1, arg2, arg3);
				break;

			case CMD_RESYNC:
				detectInterface((arg1 & MASK_IF1) != 0, (arg1 & MASK_QL) != 0);
				synced = false;
				return;
		}
	}
}

//
void daemonHWGroup() {
	daemonCmdArgs(CMD_MAP, hwGroupStart, hwGroupEnd, HW_GROUP_LOCK ? 1 : 0, 0);
}

//
bool daemonPing() {
	if (millis() - lastPing < PING_INTERVAL) {
		return false;
	}
	daemonCmd((uint8_t*)DAEMON_PING);
	synced = daemonRcvAck(10, 5, (uint8_t*)DAEMON_PONG);
	lastPing = millis();
	return synced;
}

//
void daemonCmd(uint8_t cmd[]) {
	daemonCmdArgs(cmd[0], cmd[1], cmd[2], cmd[3], 0);
}

//
void daemonPendingCmd(uint8_t a, uint8_t b, uint8_t c) {
	buffer[0] = a;
	buffer[1] = b;
	buffer[2] = c;
	Serial.write(buffer, 3);
	Serial.flush();
}

//
void daemonCmdArgs(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3,
	uint16_t bufferLen) {
	buffer[0] = cmd;
	buffer[1] = arg1;
	buffer[2] = arg2;
	buffer[3] = arg3;
	Serial.write(buffer, CMD_LENGTH + bufferLen);
	Serial.flush();
}

//
bool daemonRcvAck(uint8_t rounds, uint8_t wait, uint8_t exp[]) {
	if (daemonRcvCmd(rounds, wait)) {
		for (int ix = 0; ix < CMD_LENGTH; ix++) {
			if (buffer[CMD_LENGTH + ix] != exp[ix]) {
				return false;
			}
		}
		return true;
	}
	return false;
}

//
bool daemonRcvCmd(uint8_t rounds, uint8_t wait) {
	for (int r = 0; r < rounds; r++) {
		if (Serial.available() < CMD_LENGTH) {
			delay(wait);
		} else {
			return daemonRcv(CMD_LENGTH) == CMD_LENGTH;
		}
	}
	return false;
}

//
uint16_t daemonRcv(uint16_t bufferLen) {

	if (bufferLen == 0) { // unknown expected length, get from daemon
		if (daemonRcv(2) == 0) {
			return 0;
		};
		bufferLen = buffer[CMD_LENGTH]
			| (((uint16_t)buffer[CMD_LENGTH + 1]) << 8);
	}

	if (bufferLen > 0) {
		// don't overrun the buffer...
		uint16_t excess = bufferLen > PAYLOAD_LENGTH ? bufferLen - PAYLOAD_LENGTH : 0;
		uint16_t toRead = bufferLen - excess;
		if (Serial.readBytes(buffer + CMD_LENGTH, toRead) != toRead) {
			return 0;
		}
		// ...but still eat up all expected bytes coming in over serial
		uint8_t dummy[1];
		for (; excess > 0; excess--) {
			Serial.readBytes(dummy, 1);
		}
	}

	return bufferLen;
}


bool testBuffer() {
	return testBufferPattern(0xff)
		&&  testBufferPattern(0x00)
		&&  testBufferPattern(0xaa)
		&&  testBufferPattern(0x55);
}

bool testBufferPattern(uint8_t p) {
	for (uint16_t ix = 0; ix < BUF_LENGTH; ix++) {
		buffer[ix] = p;
	}
	for (uint16_t ix = 0; ix < BUF_LENGTH; ix++) {
		if (buffer[ix] != p) {
			return false;
		}
	}
	return true;
}

// ------------------------------------------------------------------ TIMER ---

void setTimer(int preload, TimerHandler h) {
	enableTimer(true, preload, h);
}

void stopTimer() {
	enableTimer(false, 0, NULL);
}

bool timerEnabled() {
	return TIMSK2 & (1<<TOIE2);
}

void enableTimer(bool on, int preload, TimerHandler h) {

	if (timerEnabled() == on) {
		return;
	}

	noInterrupts();
	timerHandler = h;

	if (on) {
		TCCR2A = 0;
		TCCR2B = 0;
		TIFR2 |= _BV(TOV2); // clear the overflow interrupt flag
		TCCR2B |= (1<<CS22) + (1<<CS21); // 256 pre-scaler
		if (preload < 256) { // fast
			TCNT2 = preload;
		} else { // slow
			TCNT2 = preload - 256;
			TCCR2B |= (1<<CS20); // extend pre-scaler to 1024
		}
		TIMSK2 |= (1<<TOIE2); // enable timer overflow interrupt
	} else {
		TIMSK2 &= (0<<TOIE2);
	}
	interrupts();
}

ISR(TIMER2_OVF_vect) {
	TimerHandler h = timerHandler;
	stopTimer();
	if (h != NULL) {
		h();
	}
}

// -------------------------------------------------------------------- LED ---

void errorBlink(uint8_t a, uint8_t b, uint8_t c) {
	ledWrite(false);
	while (true) {
		ledBlink(a);
		ledBlink(b);
		ledBlink(c);
		delay(1500);
	}
}

void ledBlink(uint8_t count) {
	for (uint8_t c = count; c > 0; c--) {
		ledWrite(true);
		delay(250);
		ledWrite(false);
		delay(250);
	}
	delay(1000);
}

void ledReadFlip() {
	ledFlip(MASK_LED_READ);
}

void ledRead(bool active) {
	ledActivity(MASK_LED_READ, active, LED_RW_IDLE_ON);
}

void ledWriteFlip() {
	ledFlip(MASK_LED_WRITE);
}

void ledWrite(bool active) {
	ledActivity(MASK_LED_WRITE, active, LED_RW_IDLE_ON);
}

void ledActivity(uint8_t mask, bool active, bool idleOn) {
	(idleOn ? !active : active) ? ledOn(mask) : ledOff(mask);
}

void ledOn(uint8_t led_mask) {
	PORTB |= led_mask;
}

void ledOff(uint8_t led_mask) {
	PORTB &= (~led_mask);
}

void ledFlip(uint8_t led_mask) {
	PORTB ^= led_mask;
}

// ----------------------------------------------------------------- RUMBLE ---

void rumbleGreet() {
	if (rumbleLevel > 0) {
		rumble(true);
		delay(1500);
		rumble(false);
	}
}

void rumble(bool on) {
	if (on && (rumbleLevel == 0)) {
		return;
	}
	analogWrite(PIN_RUMBLE, on ? rumbleLevel : 0);
}

// ------------------------------------------------------------- STATE SAVE ---

struct state {
	uint8_t writes; // write count at EEPROM location, for leveled writes
	uint8_t hwGroupStart; // h/w drive mapping
	uint8_t hwGroupEnd;
	uint8_t rumbleLevel; // rumble motor level
};

void loadState() {
	state* s = &state{};
	if (loadFromEEPROM(s)) {
		debugMsg('S', 'L', 1);
	} else {
		debugMsg('S', 'L', 0);
		resetState(s);
	}
	if (!HW_GROUP_LOCK) {
		setHWGroup(s->hwGroupStart, s->hwGroupEnd);
	}
	rumbleLevel = s->rumbleLevel;
}

void saveState() {
	state* s = &state{};
	s->hwGroupStart = hwGroupStart;
	s->hwGroupEnd = hwGroupEnd;
	s->rumbleLevel = rumbleLevel;
	storeToEEPROM(s);
}

void resetState(struct state* s) {
	s->hwGroupStart = HW_GROUP_START;
	s->hwGroupEnd = HW_GROUP_END;
	s->rumbleLevel = RUMBLE_LEVEL;
}

// let's use good old QL checksum ;-)
uint16_t checksumState(struct state* s) {
	uint16_t a = 0x0f0f;
	uint8_t* p = (uint8_t*)s;
	for (uint8_t ix = 0; ix < sizeof(struct state); ix++) {
		a += *(p + ix);
	}
	return a;
}

// ----------------------------------------------------------------- EEPROM ---

const int EEPROM_START = sizeof(int); // start of EEPROM is used for index

void storeToEEPROM(struct state* s) {

	s->writes++;

	int ix;
	if (s->writes > 100) {
		ix = seekEEPROM(true);
		s->writes = 0;
	} else {
		ix = seekEEPROM(false);
	}

	uint16_t sum = checksumState(s);
	EEPROM.put(ix, sum);
	EEPROM.put(ix + sizeof(sum), *s);
}

bool loadFromEEPROM(struct state* s) {

	int ix = seekEEPROM(false);

	uint16_t sum;
	EEPROM.get(ix, sum);
	EEPROM.get(ix + sizeof(sum), *s);

	return (sum == checksumState(s));
}

int seekEEPROM(bool advance) {

	int ixR;
	EEPROM.get(0, ixR);

	int ixV = validateIndex(ixR);

	if (ixR != ixV) {
		EEPROM.put(0, ixV);
	}
	if (advance) {
		ixV = validateIndex(++ixV);
		EEPROM.put(0, ixV);
	}

	return ixV;
}

int validateIndex(int ix) {
	if (ix < EEPROM_START) {
		return EEPROM_START;
	} else {
		int last = ix + sizeof(uint16_t) + sizeof(state);
		if (last > E2END) {
			return EEPROM_START;
		}
	}
	return ix;
}
