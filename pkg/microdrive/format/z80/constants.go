/*
   OqtaDrive - Sinclair Microdrive emulator
   Copyright (c) 2021, Alexander Vollschwitz

   This file is part of OqtaDrive.

   The Z80toMDR code is based on Z80onMDR_Lite, copyright (c) 2021 Tom Dalby,
   ported from C to Go by Alexander Vollschwitz. For the original C code, refer
   to:

        https://github.com/TomDDG/Z80onMDR_lite

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

package z80

/*
	port history (tags & SHA1, in reverse order):
		- v2.00		29c8ca26aa303e38dbeaab11f10b978151a8bf04
		- v1.70		9ded77aeba34dccfc86789e320b0f736ce45bc94
		- v1.60		bd6e94c6f0f8f98785135757c8e56c21c6b04fa2
		- v1.51		56f34a223a475e0931f1190265b451bd077a947f
		- v1.50		dc78c6cec168ab9c151ed07d8a1d7dab17124fe7
		- v1.41		9acb7f3eec4d54b08f2444cd0c6d0f1e73ed7d9e
		- v1.40		88b6bc26ccd267051d4af161db2110c5412eee95
		- v1.22		fde4f0d2b5dbc66f00445f56303311debc43cb4e
		- (others, not tracked)
*/

const Version = "v1"
const BGap = 128
const MaxLength = 256
const MinLength = 3

// --- basic loader -----------------------------------------------------------
const mdrBlnBRD = 16
const mdrBlnTO = 51
const mdrBlnPAP = 135  // paper/ink
const mdrBlnFCPY = 153 // final copy position
const mdrBlnCPYF = 156 // copy from, normal 0x5b00
const mdrBlnCPYX = 159 // copy times
const mdrBlnFFFD = 195 // last fffd
const mdrBlnI = 210
const mdrBlnIM = 214
const mdrBlnTS = 216
const mdrBlnJP = 219 // change if move launcher
const mdrBlnAY = 221 // start of ay array
const mdrBlnBCA = 237
const mdrBlnDEA = 239
const mdrBlnHLA = 241
const mdrBlnIX = 243
const mdrBlnIY = 245
const mdrBlnAFA = 247

var mdrBln = []byte{
	0x00, 0x00, 0x62, 0x00, 0xfd, 0x30, 0x0e, 0x00, //(0)
	0x00, 0x4f, 0x61, 0x00, 0x3a, 0xe7, 0xb0, 0x22, 0x30, 0x22, //(8) clear 24911
	0x3a, 0xf9, 0xc0, 0x30, 0x0e, 0x00, 0x00, 0x70, 0x5d, 0x00, 0x3a, 0xf1, 0x64, 0x3d, //(18) randomize usr 23920
	0xbe, 0x30, 0x0e, 0x00, 0x00, 0xd6, 0x5c, 0x00, 0x3a, //(32) let d=peek 23766
	0xeb, 0x69, 0x3d, 0xb0, 0x22, 0x30, 0x22, 0xcc, 0xb0, 0x22, 0x35, 0x22, 0x3a, 0xef, 0x2a, 0x22, //(41)
	0x6d, 0x22, 0x3b, 0x64, 0x3b, 0xc1, 0x69, 0xaf, 0x3a, 0xf9, 0xc0, 0x30, 0x0e, 0x00, 0x00, 0xb3, //(57) randomize usr 32179
	0x7d, 0x00, 0x3a, 0xf3, 0x69, 0x3a, 0xef, 0x2a, 0x22, 0x6d, 0x22, 0x3b, 0x64, 0x3b, 0x22, 0x4d, //(73)
	0x22, 0xaf, 0x3a, //(89)
	0xf9, 0xc0, 0x30, 0x0e, 0x00, 0x00, 0x9c, 0x5d, 0x00, 0x0d, //(92) randomize usr 23964
	// usr 0 code
	0x27, 0x0f, 0x99, 0x00, 0xea, //(102) line9999
	0xf3, 0x2a, 0x3d, 0x5c, 0x23, 0x36, 0x13, 0x2b, 0x36, 0x03, 0x2b, 0x36, 0x1b, 0x2b, 0x36, 0x76, //(107) usr 0
	0x2b, 0x36, 0x00, 0x2b, 0x36, 0x51, 0xf9, 0xfd, 0xcb, 0x01, 0xa6, 0x3e, 0x00, 0x32, 0x8d, 0x5c, //(123)
	0xcd, 0xaf, 0x0d, 0x3e, 0x10, 0x01, 0xfd, 0x7f, 0xed, 0x79, 0xfb, 0xc9, //(139)
	// stage 1
	0xf3, 0x21, 0x39, 0x30, 0x11, 0x00, 0x5b, 0x01, 0x36, 0x00, 0xed, //(151)
	0xb0, 0x31, 0xe2, 0x5d, 0xd9, 0x01, 0xfd, 0xff, 0xaf, 0xe1, 0xed, 0x79, 0x3c, 0x06, 0xbf, 0xed, //(162)
	0x69, 0x06, 0xff, 0xed, 0x79, 0x3c, 0x06, 0xbf, 0xed, 0x61, 0xfe, 0x10, 0x06, 0xff, 0x20, 0xe9, //(178)
	0x3e, 0x00, 0xed, 0x79, 0xc1, 0xd1, 0xe1, 0xd9, 0xdd, 0xe1, 0xfd, 0xe1, 0x08, 0xf1, 0x08, 0x3e, //(194)
	0x00, 0xed, 0x47, 0xed, 0x5e, 0x31, 0x36, 0x5b, 0xc3, 0x02, 0x5b, 0x00, 0x00, 0x00, 0x00, 0x00, //(210)
	0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //(226)
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, //(242)
}

// --- alternate loader stage 2,3 & 4 in screen -------------------------------
const launchScrSTART = 2
const launchScrLCF = 50 + 5 // bdata
const launchScrLCS = 53 + 5
const launchScrOUT = 60 + 5
const launchScrDE = 64 + 5
const launchScrBC = 67 + 5
const launchScrHL = 70 + 5
const launchScrR = 73 + 5
const launchScrSP = 78 + 5
const launchScrEI = 80 + 5
const launchScrJP = 82 + 5
const launchScrAF = 88 + 5
const launchScrDELTA = 90 + 5
const launchScrLen = 93 + 5 // for delta=3

var launchScr = []byte{
	0x11, 0x00, 0x5b, 0x18, 0x02, 0xed, 0xb0, 0x7e, 0x23, 0x4f, 0x0c, 0x28, 0x29, 0xfe, 0x20, 0x38, //(0)
	0xf4, 0xf5, 0xe6, 0xe0, 0x07, 0x07, 0x07, 0xfe, 0x07, 0x20, 0x02, 0x86, 0x23, 0xc6, 0x02, 0x4f, //(16)
	0x88, 0x91, 0x47, 0xf1, 0xe5, 0xc5, 0xe6, 0x1f, 0x47, 0x4e, 0x62, 0x6b, 0x37, 0xed, 0x42, 0xc1, //(32)
	0xed, 0xb0, 0xe1, 0x23, 0x18, 0xd1, 0x21, 0x5f, 0x40, 0x0e, 0x03, 0xed, 0xb0, 0x01, 0xfd, 0x7f, //(48)
	0x3e, 0x30, 0xed, 0x79, 0x11, 0x00, 0x00, 0x01, 0x00, 0x00, 0x21, 0x00, 0x00, 0x3e, 0x02, 0xed, //(64)
	0x4f, 0xf1, 0x31, 0x00, 0x00, 0xf3, 0xc3, 0xb7, 0xd9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //(80)
}

// --- stage 2 - printer buffer -----------------------------------------------
const nocLaunchPrtJP = 7 // where to jump to

var nocLaunchPrt = []byte{
	0xed, 0xb0, 0x7e, 0x23, 0x4f, 0x0c, 0xca, 0x36, 0x5b, 0xfe, 0x20, 0x38, 0xf3, 0xf5, 0xe6, 0xe0, //(0)
	0x07, 0x07, 0x07, 0xfe, 0x07, 0x20, 0x02, 0x86, 0x23, 0xc6, 0x02, 0x4f, 0x88, 0x91, 0x47, 0xf1, //(16)
	0xe5, 0xc5, 0xe6, 0x1f, 0x47, 0x4e, 0x62, 0x6b, 0x37, 0xed, 0x42, 0xc1, 0xed, 0xb0, 0xe1, 0x23, //(32)
	0x18, 0xd0, 0x00, 0x00, 0x00, 0x00, //(48)
}

// --- stage 3 - gap part -----------------------------------------------------
const nocLaunchIgpBDATA = 1 // bdata start, pos+16
const nocLaunchIgpLCS = 4   // last copy size=delta=3
const nocLaunchIgpDE = 14
const nocLaunchIgpCLR = 18   // amount to clear 82
const nocLaunchIgpCHR = 17   // char to clear
const nocLaunchIgpRD = 20    // stack rdata = stack-2
const nocLaunchIgpJP = 23    // jump into = stack-28
const nocLaunchIgpBEGIN = 25 // beginning of bdata
const nocLaunchIgpLen = 82   // 25 + 3 + 54 = 82bytes for delta=3

var nocLaunchIgp = []byte{
	0x21, 0x4f, 0x5b, 0x0e, 0x03, 0xed, 0xb0, 0x16, 0x5b, 0x0e, //(0)
	0x36, 0xed, 0xb0, 0x11, 0x00, 0x00, 0x01, 0x00, 0x52, 0x31, 0x64, 0x5b, 0xc3, 0x4f, 0x5b, //(10)
}

// --- stage 4 - stack part ---------------------------------------------------
const nocLaunchStkOUT = 8
const nocLaunchStkBC = 12
const nocLaunchStkHL = 15
const nocLaunchStkR = 18 // r
const nocLaunchStkEI = 22
const nocLaunchStkJP = 24
const nocLaunchStkAF = 26

var nocLaunchStk = []byte{
	0x2b, 0x71, 0x10, 0xfc, 0x01, 0xfd, 0x7f, 0x3e, 0x30, 0xed, 0x79, 0x01, 0x00, 0x00, 0x21, 0x00, //(0)
	0x00, 0x3e, 0x02, 0xed, 0x4f, 0xf1, 0xf3, 0xc3, 0xb7, 0xd9, 0x00, 0x00, //(16)
}

// --- compressed screen loader -----------------------------------------------
var scrLoad = []byte{
	0x21, 0x0b, 0x7e, 0x11, 0x00, 0x58, 0x18, 0x06, 0xcd, 0xef, 0x7d, 0x23, 0x10, 0xfa, 0x7e, 0x23, //(0)
	0x47, 0x04, 0xc8, 0xfe, 0x20, 0x38, 0xf1, 0x4f, 0xe6, 0xe0, 0x07, 0x07, 0x07, 0xfe, 0x07, 0x20, //(16)
	0x02, 0x86, 0x23, 0xc6, 0x02, 0x47, 0xe5, 0x79, 0xe6, 0x1f, 0xc6, 0x40, 0x6e, 0x67, 0xcd, 0xef, //(32)
	0x7d, 0xeb, 0xcd, 0xf1, 0x7d, 0xeb, 0x10, 0xf6, 0xe1, 0x23, 0x18, 0xd2, 0x7e, 0x12, 0x14, 0x7a, //(48)
	0xfe, 0x59, 0x38, 0x08, 0x3d, 0x07, 0x07, 0x07, 0xee, 0x82, 0x57, 0x3c, 0xe6, 0x07, 0xc0, 0xaa, //(64)
	0x1f, 0x1f, 0x1f, 0xc6, 0x4f, 0x57, 0x13, 0xc9, //(80)
}

// --- unpacker for 128k pages ------------------------------------------------
var unpack = []byte{
	0xf3, 0x3a, 0xff, 0x7d, 0x01, 0xfd, 0x7f, 0xed, 0x79, 0x21, 0x00, 0x7e, 0x11, 0x00, 0xc0, 0x43,
	0x18, 0x02, 0xed, 0xb0, 0x7e, 0x23, 0x4f, 0x0c, 0x28, 0x29, 0xfe, 0x20, 0x38, 0xf4, 0xf5, 0xe6,
	0xe0, 0x07, 0x07, 0x07, 0xfe, 0x07, 0x20, 0x02, 0x86, 0x23, 0xc6, 0x02, 0x4f, 0x88, 0x91, 0x47,
	0xf1, 0xe5, 0xc5, 0xe6, 0x1f, 0x47, 0x4e, 0x62, 0x6b, 0x37, 0xed, 0x42, 0xc1, 0xed, 0xb0, 0xe1,
	0x23, 0x18, 0xd1, 0x3e, 0x10, 0x01, 0xfd, 0x7f, 0xed, 0x79, 0xfb, 0xc9, 0x11,
}
