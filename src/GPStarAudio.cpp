/**
 *   GPStarAudio.cpp
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

#include "GPStarAudio.h"

void gpstarAudio::start(Stream& _port) {
  gpReceivedVersion = false;
  sysInfoRcvd = false;
  gpInfoReceived = false;

  GPStarSerial = &_port;

  flush();
}

void gpstarAudio::flush(void) {
  resetParser();

  for(uint8_t i = 0; i < GP_NUM_CHANNELS; i++) {
    gpChannelData[i] = 0xffff;
  }

  while(GPStarSerial->available()) {
    GPStarSerial->read();
  }
}

void gpstarAudio::serialFlush(void) {
  GPStarSerial->flush();
}

void gpstarAudio::update() {
  if(!GPStarSerial) {
    return;
  }
  
  while(GPStarSerial->available()) {
    parse(GPStarSerial->read());
  } 
}

void gpstarAudio::resetParser() {
  gpCounterRx = 0; 
  gpLengthRx = 0;
}

void gpstarAudio::parse(uint8_t d) {
  if(gpCounterRx == 0) { 
    if(d == GP_S1) {
      gpCounterRx = 1; 
      
      return; 
    }
  }

  if(gpCounterRx == 1) { 
    gpCounterRx = (d == GP_S2) ? 2:0; 

    return; 
  }

  if(gpCounterRx == 2) {
      if(d == GP_S1 || d == GP_S2 || d == GP_EM) { 
        resetParser();

        return; 
      }
      
      if(d <= GP_MSG_MAXLEN) { 
        gpLengthRx = d - 1; 
        gpCounterRx = 3; 

        return; 
      } 
      
      resetParser();

      return;
  }

  if(gpCounterRx > 2 && gpCounterRx < gpLengthRx) {
      gpRec[gpCounterRx - 3] = d;

      if(gpRec[0] == GPRCV_GPSTAR_HELLO) {
        // Skip the extra check upon the GPStar hello check.
      }

      if(d == GP_S1 || d == GP_S2 || d == GP_EM) {
        resetParser();

        return;
      }
      
      gpCounterRx++;
      
      return;
  }

  if(gpCounterRx == gpLengthRx) {
    if(d == GP_EM) {
      handleMessage();
    }

    resetParser();
  }
}

void gpstarAudio::handleMessage() {
    uint16_t track;

    switch(gpRec[0]) {
        case GPRCV_TRACK_REPORT_EX:
          track = (gpRec[2] << 8) | gpRec[1];
          currentTrack = track;
          gpCurrentTrackStatus = (gpRec[3]!=0);
          trackCounter = false;
        break;

        case GPRCV_TRACK_REPORT:
          track = ((gpRec[2] << 8) | gpRec[1]) + 1;

          if(gpRec[3] < GP_NUM_CHANNELS) {
            if(gpRec[4] == 0) { 
              if(track == gpChannelData[gpRec[3]]) { 
                gpChannelData[gpRec[3]] = 0xffff;
              }
            }
            else {
              gpChannelData[gpRec[3]] = track;
            }
          }
  
        break;

        case GPRCV_VERSION_STRING:
          memcpy(charVer, gpRec + 1, GP_VS_LEN - 1);
          charVer[GP_VS_LEN - 1] = '\0';
          gpReceivedVersion = true;
        break;

        case GPRCV_SYSTEM_INFO:
          gpChannelCount = gpRec[1];
          gpTrackCount = (gpRec[3] <<8) | gpRec[2];
          
          sysInfoRcvd = true;
        break;

        case GPRCV_GPSTAR_HELLO:
          gpChannelCount = gpRec[1];
          gpTrackCount = (gpRec[3] << 8) | gpRec[2];
          
          if(gpLengthRx >= GPSTAR_HELLO_LEN) {
            gpVersion = (gpRec[5] <<8) | gpRec[4];
          }

          gpInfoReceived = true;
        break;
    }
}

bool gpstarAudio::currentTrackStatus(uint16_t trk) {
  if(trk == currentTrack) {
    if(gpCurrentTrackStatus) {
      return true;
    }
  }

  return false;
}

__attribute__((deprecated("trackCounterReset() is deprecated. Please use isTrackCounterReset() instead.")))
bool gpstarAudio::trackCounterReset() {
  return isTrackCounterReset();
}

bool gpstarAudio::isTrackCounterReset() {
  // trackCounter is reset if it is true.
  return trackCounter;
}

__attribute__((deprecated("resetTrackCounter(bool) is deprecated and will always reset to true. Please use resetTrackCounter() without passing a parameter instead.")))
void gpstarAudio::resetTrackCounter(bool bReset) {
  // Ignore the passed parameter and reset trackCounter.
  (void)bReset;
  resetTrackCounter();
}

void gpstarAudio::resetTrackCounter() {
  // Resetting the variable means to set it to true.
  trackCounter = true;
}

void gpstarAudio::gpBuildCommand(const CommandData& cmd) {
  uint8_t txbuf[cmd.buf];
  uint8_t i_tmp = 0;

  for(uint8_t i = 0; i < cmd.buf -1; i++) {
    // Build the header.
    if(i <= 2) {
      switch(i) {
        case 0:
          txbuf[i] = GP_S1;
        break;

        case 1:
          txbuf[i] = GP_S2;
        break;

        case 2:
          txbuf[i] = (uint8_t)cmd.buf;
        break;

        default:
          //
        break;
      }
    }

    // Controls for tracks.
    if(cmd.cmd > 0 && i > 2) {
      switch(i) {
        case 3:
          txbuf[i] = cmd.cmd;
        break;

        case 4:
          txbuf[i] = cmd.code;
        break;

        case 5:
          txbuf[i] = (uint8_t)cmd.trk;
        break;

        case 6:
          txbuf[i] = (uint8_t)(cmd.trk >> 8);
        break;

        case 7:
          if(cmd.useRapid) {
            txbuf[i] = (uint8_t)cmd.rapid_time;
          }
          else {
            txbuf[i] = cmd.lock;
          }
        break;

        case 8:
          if(cmd.useTrack2) {
            txbuf[i] = (uint8_t)cmd.trk2;
          }
          else if(cmd.useRapid) {
            txbuf[i] = (uint8_t)(cmd.rapid_time >> 8);
          }
        break;

        case 9:
          if(cmd.useTrack2) {
            txbuf[i] = (uint8_t)(cmd.trk2 >> 8);
          }
        break;

        case 10:
          if(cmd.useTrack2) {
            txbuf[i] = cmd.loop_trk2;
          }
        break;

        case 11:
          if(cmd.useDelay2) {
            txbuf[i] = (uint8_t)cmd.trk2_start_time;
          }
        break;

        case 12:
          if(cmd.useDelay2) {
            txbuf[i] = (uint8_t)(cmd.trk2_start_time >> 8);
          }
        break;

        case 13:
          if(cmd.useDelay1) {
            txbuf[i] = (uint8_t)cmd.trk1_start_time;
          }
        break;

        case 14:
          if(cmd.useDelay1) {
            txbuf[i] = (uint8_t)(cmd.trk1_start_time >> 8);
          }
        break;
        
        default:
          //
        break;
      }
    }
    else {
      // Other commands
      switch(i) {
        case 3:
          txbuf[i] = cmd.code;
        break;

        case 4:
          switch(cmd.code) {
            case GPCMD_TRACK_FADE:
            case GPCMD_TRACK_VOLUME:
            case GPCMD_SAMPLERATE_OFFSET:
            case GPCMD_MASTER_VOLUME:
            case GPCMD_GET_TRACK_STATUS:
              txbuf[i] = (uint8_t)cmd.trk;
            break;

            case GPCMD_SET_TRIGGER_BANK:
            case GPCMD_SET_REPORTING:
            case GPCMD_AMP_POWER:
              txbuf[i] = cmd.lock;
            break;
          }
        break;

        case 5:
          switch(cmd.code) {
            case GPCMD_TRACK_FADE:
            case GPCMD_TRACK_VOLUME:
            case GPCMD_SAMPLERATE_OFFSET:
            case GPCMD_MASTER_VOLUME:
            case GPCMD_GET_TRACK_STATUS:
              txbuf[i] = (uint8_t)(cmd.trk >> 8);
            break;
          }
        break;
        
        case 6:
          switch(cmd.code) {
            case GPCMD_TRACK_FADE:
            case GPCMD_TRACK_VOLUME:
              txbuf[i] = (uint8_t)cmd.trk2;
            break;
          }
        break;

        case 7:
          switch(cmd.code) {
            case GPCMD_TRACK_FADE:
            case GPCMD_TRACK_VOLUME:
              txbuf[i] = (uint8_t)(cmd.trk2 >> 8);
            break;
          }
        break;

        case 8:
          switch(cmd.code) {
            case GPCMD_TRACK_FADE:
              txbuf[i] = (uint8_t)cmd.trk1_start_time;
            break;
          }
        break;

        case 9:
          switch(cmd.code) {
            case GPCMD_TRACK_FADE:
              txbuf[i] = (uint8_t)(cmd.trk1_start_time >> 8);
            break;
          }
        break;

        case 10:
          switch(cmd.code) {
            case GPCMD_TRACK_FADE:
              txbuf[i] = cmd.useDelay1;
            break;
          }
        break;

        default:
          //
        break;
      }
    }
    

    i_tmp++;
  }

  txbuf[i_tmp] = GP_EM;

  GPStarSerial->write(txbuf, cmd.buf);
}

void gpstarAudio::trackPlayingStatus(uint16_t trk) {
  CommandData cmd {};
  cmd.buf = 7;
  cmd.trk = trk;
  cmd.code = GPCMD_GET_TRACK_STATUS;

  gpBuildCommand(cmd);
}

bool gpstarAudio::isTrackPlaying(uint16_t trk) {
  update();

  for(uint8_t i = 0; i < GP_NUM_CHANNELS; i++) {
    if(gpChannelData[i] == trk) {
      return true;
    }
  }

  return false;
}

void gpstarAudio::masterGain(int16_t gain) {
  CommandData cmd {};
  cmd.buf = 7;
  cmd.trk = gain;
  cmd.code = GPCMD_MASTER_VOLUME;

  gpBuildCommand(cmd);
}

void gpstarAudio::setAmpPwr(bool enable) {
  CommandData cmd {};
  cmd.buf = 6;
  cmd.code = GPCMD_AMP_POWER;
  cmd.lock = enable;

  gpBuildCommand(cmd);
}

void gpstarAudio::setReporting(bool enable) {
  CommandData cmd {};
  cmd.buf = 6;
  cmd.code = GPCMD_SET_REPORTING;
  cmd.lock = enable;

  gpBuildCommand(cmd);
}

bool gpstarAudio::getVersion(char *gpTmp) {
  update();

  if(!gpReceivedVersion) {
    return false;
  }

  for(uint8_t i = 0; i < (GP_VS_LEN - 1); i++) {
    gpTmp[i] = charVer[i];
  }

  return true;
}

uint16_t gpstarAudio::getVersionNumber(void) {
  return gpVersion;
}

uint16_t gpstarAudio::getNumTracks(void) {
  update();

  return gpTrackCount;
}

void gpstarAudio::trackPlaySolo(uint16_t trk) {
  CommandData cmd {};
  cmd.buf = 8;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL;
  cmd.code = GPTRACK_PLAY_SOLO;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackPlaySolo(uint16_t trk, bool lock) {
  CommandData cmd {};
  cmd.buf = 9;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL_EX;
  cmd.code = GPTRACK_PLAY_SOLO;
  cmd.lock = lock;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackPlaySolo(uint16_t trk, bool lock, uint16_t i_trk_start_delay) {
  CommandData cmd {};
  cmd.buf = 11;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL_CACHE;
  cmd.code = GPTRACK_PLAY_SOLO;
  cmd.lock = lock;
  cmd.useDelay1 = true;
  cmd.trk1_start_time = i_trk_start_delay;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackPlaySolo(uint16_t trk, bool lock, uint16_t i_trk_start_delay, uint16_t trk2, bool loop_trk2, uint16_t trk2_start_time) {
  CommandData cmd {};
  cmd.buf = 16;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL_QUEUE;
  cmd.code = GPTRACK_PLAY_SOLO;
  cmd.lock = lock;
  cmd.useDelay1 = true;
  cmd.trk1_start_time = i_trk_start_delay;
  cmd.useDelay2 = true;
  cmd.trk2 = trk2;
  cmd.loop_trk2 = loop_trk2;
  cmd.trk2_start_time = trk2_start_time;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackPlayPoly(uint16_t trk) {
  CommandData cmd {};
  cmd.buf = 8;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL;
  cmd.code = GPTRACK_PLAY_POLY;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackPlayPoly(uint16_t trk, bool lock) {
  CommandData cmd {};
  cmd.buf = 9;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL_EX;
  cmd.code = GPTRACK_PLAY_POLY;
  cmd.lock = lock;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackPlayPoly(uint16_t trk, bool lock, uint16_t i_trk_start_delay) {
  if(i_trk_start_delay < 1) {
    trackPlayPoly(trk, lock);
  }
  else {
    CommandData cmd {};
    cmd.buf = 11;
    cmd.trk = trk;
    cmd.cmd = GPCMD_TRACK_CONTROL_CACHE;
    cmd.code = GPTRACK_PLAY_POLY;
    cmd.lock = lock;
    cmd.useDelay1 = true;
    cmd.trk1_start_time = i_trk_start_delay;

    gpBuildCommand(cmd);
  }
}

void gpstarAudio::trackPlayPoly(uint16_t trk, bool lock, uint16_t i_trk_start_delay, uint16_t trk2, bool loop_trk2, uint16_t trk2_start_time) {
  CommandData cmd {};
  cmd.buf = 16;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL_QUEUE;
  cmd.code = GPTRACK_PLAY_POLY;
  cmd.lock = lock;
  cmd.useDelay1 = true;
  cmd.trk1_start_time = i_trk_start_delay;
  cmd.useDelay2 = true;
  cmd.trk2 = trk2;
  cmd.loop_trk2 = loop_trk2;
  cmd.trk2_start_time = trk2_start_time;

  gpBuildCommand(cmd);  
}

void gpstarAudio::trackLoad(uint16_t trk) {
  CommandData cmd {};
  cmd.buf = 8;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL;
  cmd.code = GPTRACK_LOAD;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackLoad(uint16_t trk, bool lock) {
  CommandData cmd {};
  cmd.buf = 9;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL_EX;
  cmd.code = GPTRACK_LOAD;
  cmd.lock = lock;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackStop(uint16_t trk) {
  CommandData cmd {};
  cmd.buf = 8;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL;
  cmd.code = GPTRACK_STOP;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackPause(uint16_t trk) {
  CommandData cmd {};
  cmd.buf = 8;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL;
  cmd.code = GPTRACK_PAUSE;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackResume(uint16_t trk) {
  CommandData cmd {};
  cmd.buf = 8;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL;
  cmd.code = GPTRACK_RESUME;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackLoop(uint16_t trk, bool enable) {
  CommandData cmd {};
  cmd.buf = 9;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL_EX;
  cmd.code = enable ? GPTRACK_LOOP_ON : GPTRACK_LOOP_OFF;
  cmd.lock = enable;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackRapidPlay(uint16_t trk, uint16_t i_rapid_play) {
  CommandData cmd {};
  cmd.buf = 10;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL;
  cmd.code = GPTRACK_RAPID_PLAY;
  cmd.useRapid = true;
  cmd.rapid_time = i_rapid_play;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackRapidDelay(uint16_t trk, uint16_t i_rapid_delay) {
  CommandData cmd {};
  cmd.buf = 10;
  cmd.trk = trk;
  cmd.cmd = GPCMD_TRACK_CONTROL;
  cmd.code = GPTRACK_RAPID_DELAY;
  cmd.useRapid = true;
  cmd.rapid_time = i_rapid_delay;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackQueueClear() {
  CommandData cmd {};
  cmd.buf = 5;
  cmd.code = GPCMD_TRACK_QUEUE_CLEAR;

  gpBuildCommand(cmd);
}

void gpstarAudio::stopAllTracks(void) {
  CommandData cmd {};
  cmd.buf = 5;
  cmd.code = GPCMD_STOP_ALL;

  gpBuildCommand(cmd);
}

void gpstarAudio::resumeAllInSync(void) {
  CommandData cmd {};
  cmd.buf = 5;
  cmd.code = GPCMD_SAMPLERATE_OFFSET;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackGain(uint16_t trk, int16_t gain) {
  CommandData cmd {};
  cmd.buf = 9;
  cmd.trk = trk;
  cmd.code = GPCMD_TRACK_VOLUME;
  cmd.trk2 = gain;

  gpBuildCommand(cmd);
}

void gpstarAudio::trackFade(uint16_t trk, int16_t gain, uint16_t time, bool stopFlag) {
  CommandData cmd {};
  cmd.buf = 12;
  cmd.trk = trk;
  cmd.code = GPCMD_TRACK_FADE;
  cmd.trk2 = gain;
  cmd.trk1_start_time = time;
  cmd.useDelay1 = stopFlag;

  gpBuildCommand(cmd);
}

void gpstarAudio::samplerateOffset(int16_t offset) {
  CommandData cmd {};
  cmd.buf = 7;
  cmd.trk = offset;
  cmd.code = GPCMD_SAMPLERATE_OFFSET;

  gpBuildCommand(cmd);
}

void gpstarAudio::setTriggerBank(uint8_t bank) {
  CommandData cmd {};
  cmd.buf = 6;
  cmd.lock = bank;
  cmd.code = GPCMD_SET_TRIGGER_BANK;

  gpBuildCommand(cmd);
}

// Turn on or off the LED on GPStar Audio. Default is on.
void gpstarAudio::gpstarLEDStatus(bool enable) {
  CommandData cmd {};
  cmd.buf = 5;
  cmd.code = enable ? GPCMD_LED_ON : GPCMD_LED_OFF;

  gpBuildCommand(cmd);
}

// Turn on track short overload or turn it off.
void gpstarAudio::gpstarShortTrackOverload(bool enable) {
  CommandData cmd {};
  cmd.buf = 5;
  cmd.code = enable ? GPCMD_SHORT_OVERLOAD_ON : GPCMD_SHORT_OVERLOAD_OFF;

  gpBuildCommand(cmd);
}

// Turn on track force or turn it off.
void gpstarAudio::gpstarTrackForce(bool enable) {
  CommandData cmd {};
  cmd.buf = 5;
  cmd.code = enable ? GPCMD_TRACK_FORCE_ON : GPCMD_TRACK_FORCE_OFF;

  gpBuildCommand(cmd);
}

void gpstarAudio::requestVersionString(void) {
  CommandData cmd {};
  cmd.buf = 5;
  cmd.code = GPCMD_GET_VERSION;

  gpBuildCommand(cmd);
}

void gpstarAudio::requestSystemInfo(void) {
  CommandData cmd {};
  cmd.buf = 5;
  cmd.code = GPCMD_GET_SYS_INFO;

  gpBuildCommand(cmd);
}

void gpstarAudio::hello(void) {
  CommandData cmd {};
  cmd.buf = 5;
  cmd.code = GPCMD_GET_GPSTAR_HELLO;

  gpBuildCommand(cmd);
}

bool gpstarAudio::wasSysInfoRcvd(void) {
  update();

  return sysInfoRcvd;
}

bool gpstarAudio::gpstarAudioHello(void) {
  update();

  return gpInfoReceived;
}