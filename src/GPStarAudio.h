/**
 *   GPStarAudio.h
 *   Copyright (C) 2026 GPStar Technologies <contact@gpstartechnologies.com>
 * 
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, see <https://www.gnu.org/licenses/>.
 */

#pragma once
#include "Arduino.h"

// ---- GPCMD ----
#define GPCMD_GET_VERSION          1
#define GPCMD_GET_SYS_INFO         2
#define GPCMD_TRACK_CONTROL        3
#define GPCMD_STOP_ALL             4
#define GPCMD_MASTER_VOLUME        5
#define GPCMD_TRACK_VOLUME         8
#define GPCMD_AMP_POWER            9
#define GPCMD_TRACK_FADE          10
#define GLCMD_RESUME_ALL_SYNC     11
#define GPCMD_SAMPLERATE_OFFSET   12
#define GPCMD_TRACK_CONTROL_EX    13
#define GPCMD_SET_REPORTING       14
#define GPCMD_SET_TRIGGER_BANK    15
#define GPCMD_GET_TRACK_STATUS    16
#define GPCMD_GET_GPSTAR_HELLO    17
#define GPCMD_LED_ON              18
#define GPCMD_LED_OFF             19
#define GPCMD_SHORT_OVERLOAD_ON   20
#define GPCMD_SHORT_OVERLOAD_OFF  21
#define GPCMD_TRACK_FORCE_ON      22
#define GPCMD_TRACK_FORCE_OFF     23
#define GPCMD_TRACK_QUEUE_CLEAR   25
#define GPCMD_TRACK_CONTROL_QUEUE 26
#define GPCMD_TRACK_CONTROL_CACHE 27

// ---- GPRCV ----
#define GPTRACK_PLAY_SOLO         0
#define GPTRACK_PLAY_POLY         1
#define GPTRACK_PAUSE             2
#define GPTRACK_RESUME            3
#define GPTRACK_STOP              4
#define GPTRACK_LOOP_ON           5
#define GPTRACK_LOOP_OFF          6
#define GPTRACK_LOAD              7
#define GPTRACK_RAPID_PLAY        8
#define GPTRACK_RAPID_DELAY       9

// ---- GPRCV ----
#define GPRCV_VERSION_STRING      129
#define GPRCV_SYSTEM_INFO         130
#define GPRCV_STATUS              131
#define GPRCV_TRACK_REPORT        132
#define GPRCV_TRACK_REPORT_EX     133
#define GPRCV_GPSTAR_HELLO        134

// ---- Misc ----
#define GP_MSG_MAXLEN             32
#define GP_NUM_CHANNELS           14
#define GP_VS_LEN                 21
#define GPSTAR_HELLO_LEN          9

// ---- Protocol ----
#define GP_S1                     0xf0
#define GP_S2                     0xaa
#define GP_EM                     0x55

struct CommandData {
  uint8_t  buf = 0;
  uint16_t trk = 0;
  uint8_t  cmd = 0;
  uint8_t  code = 0;
  uint8_t  bank = 0;
  bool lock = false;

  bool useDelay1 = false;
  uint16_t trk1_start_time = 0;

  bool useTrack2 = false;
  uint16_t trk2 = 0;

  bool loop_trk2 = false;

  bool useDelay2 = false;
  uint16_t trk2_start_time = 0;

  bool useRapid = false;
  uint16_t rapid_time = 0;
};

class gpstarAudio {
public:
  gpstarAudio() {;}
  ~gpstarAudio() {;}
  void start(Stream& _port);
  void update(void);
  void flush(void);
  
  void parse(uint8_t b);
  void resetParser();
  void handleMessage();

  void setReporting(bool enable);
  void setAmpPwr(bool enable);
  bool getVersion(char *gpTmp);
  uint16_t getNumTracks(void);
  uint16_t getVersionNumber(void);
  bool isTrackPlaying(uint16_t trk);
  void masterGain(int16_t gain);
  void stopAllTracks(void);
  void resumeAllInSync(void);
  void trackPlaySolo(uint16_t trk);
  void trackPlaySolo(uint16_t trk, bool lock);
  void trackPlaySolo(uint16_t trk, bool lock, uint16_t i_trk_start_delay);
  void trackPlaySolo(uint16_t trk, bool lock, uint16_t i_trk_start_delay, uint16_t trk2, bool loop_trk2, uint16_t trk2_start_time);
  void trackPlayPoly(uint16_t trk);
  void trackPlayPoly(uint16_t trk, bool lock);
  void trackPlayPoly(uint16_t trk, bool lock, uint16_t i_trk_start_delay);
  void trackPlayPoly(uint16_t trk, bool lock, uint16_t i_trk_start_delay, uint16_t trk2, bool loop_trk2, uint16_t trk2_start_time);
  void trackRapidPlay(uint16_t trk, uint16_t i_rapid_play);
  void trackRapidDelay(uint16_t trk, uint16_t i_rapid_delay);
  void trackQueueClear(void);
  void trackLoad(uint16_t trk);
  void trackLoad(uint16_t trk, bool lock);
  void trackStop(uint16_t trk);
  void trackPause(uint16_t trk);
  void trackResume(uint16_t trk);
  void trackLoop(uint16_t trk, bool enable);
  void trackGain(uint16_t trk, int16_t gain);
  void trackFade(uint16_t trk, int16_t gain, uint16_t time, bool stopFlag = false);
  void samplerateOffset(int16_t offset);
  void setTriggerBank(uint8_t bank);
  void trackPlayingStatus(uint16_t trk);
  bool currentTrackStatus(uint16_t trk);
  bool trackCounterReset(void);
  void resetTrackCounter(bool bReset);
  bool isTrackCounterReset(void);
  void resetTrackCounter(void);
  void serialFlush(void);
  void requestVersionString(void);
  void requestSystemInfo(void);
  void hello(void);
  void gpstarLEDStatus(bool enable);
  void gpstarShortTrackOverload(bool enable);
  void gpstarTrackForce(bool enable);
  bool wasSysInfoRcvd(void);
  bool gpstarAudioHello(void);

private:
  void gpBuildCommand(const CommandData& cmd);
  Stream* GPStarSerial;

  uint8_t gpRec[GP_MSG_MAXLEN];
  uint8_t gpChannelCount=0;
  uint8_t gpCounterRx = 0;
  uint8_t gpLengthRx = 0;

  uint16_t gpChannelData[GP_NUM_CHANNELS];
  uint16_t gpTrackCount = 0;
  uint16_t gpVersion = 0;
  uint16_t currentTrack;

  bool gpReceivedVersion;
  bool sysInfoRcvd;
  bool gpInfoReceived;
  bool gpCurrentTrackStatus;
  bool trackCounter;

  char charVer[GP_VS_LEN];
};