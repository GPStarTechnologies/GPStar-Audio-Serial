
## <img src='images/gpstar_logo.png' width=50 align="left"/> GPStar Audio - Serial

This repository contains the serial communication control library for the [GPStar Audio](https://gpstartechnologies.com/products/gpstar-audio) and [GPStar Audio XL](https://gpstartechnologies.com/products/gpstar-audio-xl) series of audio boards from GPStar Technologies. They are compatible with Arduino and other microcontrollers and in the case of the XL version, sounds can be triggered from the bank of 20 available GPIO pins.

[![](images/gpstar_audio.png)](https://gpstartechnologies.com/products/gpstar-audio)

GPStar Audio and GPStar Audio XL are polyphonic audio players capable of playing up to 14 stereo WAV files at the same time, layering them, mixing and crossfading with independent track control of volume, pause, resume.

It has a 3.5mm stereo auxilliary jack, and its onboard stereo amplifier is capable of driving 2 speakers (2.5W @ 4Ω or 1.25W @ 8Ω per channel) from the onboard amplifier. Powered with 5V to the 5V and GND pins it can be controlled over a serial interface with an Arduino or any other microcontroller. It can be used for a variety of projects such as toys, musical instruments, props or any other embedded projects.

Example code to demonstrate some of the GPStar Audio's features can be found in the examples folder.

## Serial commands ##

**GPStarAudio.start(SerialObject)** - This must be called first to setup the serial communications. You must initalise the serial object and pass it to the function. Example: `Serial.begin(57600);` `GPStarAudio.start(Serial);`

**GPStarAudio.flush()** - Flushes all data from the GPStarAudio instance. Note this is called automatically in `GPStarAudio.start()` and so should not be necessary after initialisation.

**GPStarAudio.serialFlush()** - Flushes the serial buffer of whichever serial UART is associated with this GPStarAudio instance.

**GPStarAudio.hello()** - Call this to have the GPStar Audio respond with a hello request. See next method to check for the return value.

**GPStarAudio.gpstarAudioHello()** - Returns a `bool` for whether the `GPStarAudio.hello()` command was accepted by the GPStar Audio and if `true` populates the variable for the total number of tracks on the micro SD card and retrieves the current GPStar Audio firmware version (see `GPStarAudio.getNumTracks()` below).

**GPStarAudio.getVersionNumber()** - Returns a `uint16_t` of the GPStar Audio firmware version. You will want to call GPStarAudioHello() first to get the data. `Requires GPStar Audio Firmware v1.04 or higher.`

**GPStarAudio.update()** - Calling this will process any incoming serial data from GPStar Audio. If you are using `GPStarAudio.currentTrackStatus()` calls, then you will want to call this often.

**GPStarAudio.getNumTracks()** - This returns a `uint16_t` of the number of tracks on the micro SD card. Note that you must have called `hello()` followed by `GPStarAudio.gpstarAudioHello()` first for this to return a valid value.

**GPStarAudio.masterGain(int16_t gain)** - This sets the master gain (in dB) of the audio output amplifier. The range is `-59` (quietest) to `24` (loudest). Note that `24` is only achievable using the speaker amplifier. If using the headphone jack, the output amplifier gain has a maximum of `18`.

**GPStarAudio.trackPlaySolo(uint16_t trk)** - This will stop any tracks that are currently playing and play the selected track number provided.

**GPStarAudio.trackPlaySolo(uint16_t trk, bool lock)** - Same as above with the following addition: If `lock` is set to `true`, the track will not not ever be unloaded or stopped from the channel it has acquired if the maximum number of channels are in use unless you manually tell the track to stop.

**GPStarAudio.trackPlaySolo(uint16_t trk, bool lock, uint16_t i_trk_start_delay)** - Same as above with the following addition: Specifying `i_trk_start_delay` will specify the number of milliseconds to delay playback for, giving time to "preload" the audio file into memory. This may be useful on low-performance SD cards. `Requires GPStar Audio Firmware v1.04 or higher.`

**GPStarAudio.trackPlaySolo(uint16_t trk, bool lock, uint16_t i_trk_start_delay, uint16_t trk2, bool loop_trk2, uint16_t trk2_start_time)** - Same as above with the following addition: Setting `trk2` allows you to "queue" a second audio file to play once the first finishes playing, allowing seamless transition between tracks. `loop_trk2` defines whether the second track will loop forever, and `trk2_start_time` specifies how many milliseconds before the end of the first track the second track will begin playing. `Requires GPStar Audio Firmware v1.04 or higher.`

**GPStarAudio.trackPlayPoly(uint16_t trk)** - This will play the provided track number, mixing and overlaying it with any other tracks that are currently playing.

**GPStarAudio.trackPlayPoly(uint16_t trk, bool lock)** - Same as above with the following addition: If `lock` is set to `true`, the track will not not ever be unloaded or stopped from the channel it has acquired if the maximum number of channels are in use unless you manually tell the track to stop.

**GPStarAudio.trackPlayPoly(uint16_t trk, bool lock, uint16_t i_trk_start_delay)** - Same as above with the following addition: Specifying `i_trk_start_delay` will specify the number of milliseconds to delay playback for, giving time to "preload" the audio file into memory. This may be useful on low-performance SD cards. `Requires GPStar Audio Firmware v1.04 or higher.`

**GPStarAudio.trackPlayPoly(uint16_t trk, bool lock, uint16_t i_trk_start_delay, uint16_t trk2, bool loop_trk2, uint16_t trk2_start_time)** - Same as above with the following addition: Setting `trk2` allows you to "queue" a second audio file to play once the first finishes playing, allowing seamless transition between tracks. `loop_trk2` defines whether the second track will loop forever, and `trk2_start_time` specifies how many milliseconds before the end of the first track the second track will begin playing. `Requires GPStar Audio Firmware v1.04 or higher.`

**GPStarAudio.trackRapidPlay(uint16_t trk, uint16_t i_rapid_delay)** - This will tell GPStar Audio to fill two channels with the same track, locks both and sets both to loop at a cycle rate specified by `i_rapid_delay`. The first channel plays first and when the rapid delay timer ends, it will trigger the second channel to start playing while channel 1 continues to play. When the timer for channel 2 ends, it will tell channel 1 to rewind and play again from the start. This process will continue until you tell the track to stop or turn off track looping for the track which allows the channels to finish playing out. `Requires GPStar Audio Firmware v1.09 or higher.`

**GPStarAudio.trackRapidDelay(uint16_t trk, uint16_t i_rapid_delay)** - This updates the rapid delay timer length for the track that is using `GPStarAudio.trackRapidPlay()`. `Requires GPStar Audio Firmware v1.09 or higher.`

**GPStarAudio.trackQueueClear()** - If `GPStarAudio.trackPlaySolo()` or `GPStarAudio.trackPlayPoly()` are called with the `trk2` parameters, calling this afterwards will clear out the queue to prevent the second track from playing when the first track finishes playback. `Requires GPStar Audio Firmware v1.04 or higher.`

**GPStarAudio.trackStop(uint16_t trk)** - This stops the provided track number if it is currently playing and frees the channel it was using.

**GPStarAudio.trackPause(uint16_t trk)** - This will pause the provided track number if it is currently playing. The track will still stay loaded and hold the current channel it is allocated.

**GPStarAudio.trackResume(uint16_t trk)** - This will resume the provided track if it is currently paused.

**GPStarAudio.trackLoop(uint16_t trk, bool enable)** - This will set a track to loop or not loop. When set to loop, the track will automatically replay itself until you call `GPStarAudio.trackStop(int t)` or set the the track looping off. Set to true to enable looping or false to disable looping.

**GPStarAudio.trackGain(uint16_t trk, int16_t gain)** - This will set the volume of the provided track number. The volume range is `-59` (quietest) to `0` (loudest). This only affects the volume of this track within the mix, not the gain of the output amplifier.

**GPStarAudio.trackFade(uint16_t trk, int16_t gain, uint16_t time, bool stopFlag)** - This will fade the currently playing provided track number. The track volume will logarithmically fade to the target gain you provide from whatever volume the track is already playing at. The `stopFlag` by default is `false` which will keep the track playing after the fade. Setting the `stopFlag` to `true` will make the track stop playing and free its channel after the fade has finished.

**GPStarAudio.trackLoad(uint16_t trk)** - This will assign the provided track number to a channel paused. This can be useful when combined with the `GPStarAudio.resumeAllInSync()` method listed below.

**GPStarAudio.trackLoad(uint16_t trk, bool lock)** - Same as above with the following addition: If `lock` is set to `true`, the track will not not ever be unloaded from the channel it has acquired if the maximum number of channels are in use unless you manually tell the track to stop. This function can be useful when combined with the `GPStarAudio.resumeAllInSync()` method listed below.

**GPStarAudio.stopAllTracks()** - This will stop all tracks that are currently playing and free all channels.

**GPStarAudio.resumeAllInSync()** - This will resume all tracks which are currently paused at the exact same time.

**GPStarAudio.samplerateOffset(uint16_t offset)** - This sets the sample-rate offset of the main output mix. The range for the offset is `-32767` to `32676`, giving a speed range of 1/2x to 2x or a pitch range of down one octave to up one octave. If audio is playing you will hear the result immediately. If audio is not playing, the new sample-rate offset will be used the next time a track is started.

**GPStarAudio.gpstarLEDStatus(bool status)** - You can turn off the LED status indicator by setting `status` to `false`. Passing `true` enables the LED again. By default the LED on GPStar Audio flashes and blinks to provide various status updates.

**GPStarAudio.gpstarShortTrackOverload(bool status)** - Enabled by default. GPStar Audio will detect mulitple versions of the same sound playing in succession and prevent it from overloading and taking too many audio channels, instead replaying the file to save system resources.

**GPStarAudio.gpstarTrackForce(bool status)** - Disabled by default. When enabled, GPStar Audio will forcibly take a audio channel when requested to play a new track even if all channels in use and locked.

**GPStarAudio.trackPlayingStatus(uint16_t trk)** - This will ask GPStar Audio if the provided track number is playing. After calling this method, call the `GPStarAudio.currentTrackStatus(uint16_t trk)` method to find out if the track is playing or not.

**GPStarAudio.currentTrackStatus(uint16_t trk)** - This will retrieve the status of a the provided track number if it is playing. You will want to use the `GPStarAudio.trackPlayingStatus(uint16_t trk)` method first to ask if the provided track is playing, then call this method soon after to retrieve the response.

**GPStarAudio.resetTrackCounter()** - Resets the flag for the track status counter. Useful to call this before calling `GPStarAudio.trackPlayingStatus()`, as you can then call `GPStarAudio.isTrackCounterReset()` to determine if the new track playing status information has been received.

**GPStarAudio.isTrackCounterReset()** - This returns a `bool` of whether the internal track counter variable has been reset. It will return `true` after `GPStarAudio.resetTrackCounter()` above has been called, and will return `false` once the response from `GPStarAudio.trackPlayingStatus()` has been received.

**GPStarAudio.setReporting(bool enable)** - When enabled, GPStar Audio will report back and send updates about the status of tracks playing or stopping.

**GPStarAudio.isTrackPlaying(uint16_t trk)** - Determine if a track is currently playing or not. `GPStarAudio.setReporting()` must be enabled for this to work.

**GPStarAudio.setAmpPwr(bool enable)** - Provided for backwards compatibility with existing polyphonic audio boards, but has no effect on GPStar Audio (which uses a headphone sense circuit to dynamically switch between the headphone and speaker amplifiers).

**GPStarAudio.setTriggerBank(uint8_t bank)** - Provided for backwards compatibility with existing polyphonic audio boards, but has no effect on GPStar Audio (which does not support creation of audio banks).

### Legacy Commands (deprecated)
**GPStarAudio.resetTrackCounter(bool bReset)** - Identical to `GPStarAudio.resetTrackCounter()` above as the boolean parameter is ignored (always set to `true`). `Please call this without a parameter instead.`

**GPStarAudio.trackCounterReset()** - Identical to `GPStarAudio.isTrackCounterReset()` above. This command was deprecated because it was too easily confused with `GPStarAudio.resetTrackCounter()`. `Please use GPStarAudio.isTrackCounterReset() instead.`

**GPStarAudio.wasSysInfoRcvd()** - Returns a `bool` for whether the `GPStarAudio.requestSystemInfo()` command was accepted by the GPStar Audio and if `true` populates the variable for the total number of tracks on the micro SD card (see `GPStarAudio.getNumTracks()`). This function is provided for backwards compatibility with other polyphonic audio boards.

**GPStarAudio.requestSystemInfo()** - Call this to have the a responds with an `RSP_SYSTEM_INFO` packet. This function is provided for backwards compatibility with other polyphonic audio boards. `This is no longer used by GPStar Audio.`

**GPStarAudio.requestVersionString()** - Call this to have the GPStar Audio respond with an `RSP_VERSION_STRING` packet containing the current software version number. See next method to check for the return value. `This is no longer used by GPStar Audio.`

**GPStarAudio.getVersion(char\* version)** - Returns a `bool` for whether the `RSP_VERSION_STRING` packet was received. Pass a `char` array of size `VERSION_STRING_LEN` as the parameter to store the version string value.  `This is no longer used by GPStar Audio.`

**GPStarAudio.setTriggerBank(uint8_t bank)** - Provided for backwards compatibility with existing polyphonic audio boards, but has no effect on GPStar Audio (which does not support creation of audio banks).

## <img src='images/gpstar_logo.png' width=50 align="left"/>GPStar Audio - Connection Details

![](images/GPStarAudioPCB.png)

### Standard Connection UART

| Pins | Notes |
|------|-------|
| DTR/TX/RX/5V/GND/GND | This connection serves as the main power source and communication but it also serves as a UART programming header for software updates. Firmware can be flashed here with a standard 5V FTDI Basic. Connect a regulated 5V power source to the 5V and either GND pin to provide power to the device. |

---

### Special Connectors & Switches

| Label | Notes |
|-------|-------|
| BOOT/LOAD | A slide switch button. When set to BOOT the system will operate normally. When set to LOAD the system enters programming mode and allows you to flash updated firmware over the UART connection. Make sure it is set to BOOT afterwards for it to operate. |
| SPKR-R | Right Speaker output from the onboard stereo amplifier. Capable of powering either a 4Ω 2.5W or 8Ω 1.25W speaker at 5V. |
| SPKR-L | Left Speaker output from the onboard stereo amplifier. Capable of powering either a 4Ω 2.5W or 8Ω 1.25W speaker at 5V. |
| AUX | ****Not labelled**** The onboard stereo auxiliary headphone jack outputs stereo sound and can be fed to any type of amplifier or audio device with a 3.5mm auxiliary input jack. When a cable is inserted the system will turn off the onboard stereo amplifier and output sound from this port instead.

#### If you are using the onboard stereo amplifier with only one speaker for a mono output setup, connect the single speaker to either SPKR-R or to SPKR-L only.

#### The onboard stereo amplifier is capable of powering either a 4Ω 2.5W or 8Ω 1.25W speaker at 5V from each channel.

---

### Optional Connectors

| Label | Pins | Notes |
|-------|------|-------|
| ST-LINK | GND/3.3V/SWCLK/SWDIO | ****Not Labelled**** These 4 pins grouped together can be connected to a ST-LINK for debugging or as a alternative way to flash updated firmware. |
| TX2/RX2 | TX2/RX2 | Alternative serial communication port used for debugging. |
| TEST |  | A button located on the lower right corner of the board. Pressing this button will play the first track on the micro SD card. |

## <img src='images/gpstar_logo.png' width=50 align="left"/>GPStar Audio XL - Connection Details

![](images/GPStarAudioXLPCB.png)

### Standard Connection UART

| Pins | Notes |
|------|-------|
| DTR/TX/RX/5V/GND/GND | This connection serves as the main 5V power source and communication but it also serves as a UART programming header for software updates. Firmware can be flashed here with a standard 5V FTDI Basic. Connect a regulated 5V power source to the 5V and either GND pin to provide power to the device. |

---

### Special Connectors & Switches

| Label | Notes |
|-------|-------|
| 5V USB-C | A standard USB-C port for which you can provide 5V power. The recommended way to power GPStar Audio XL. |
| VIN (12V-40V) | You can power the device through the DC barrel jack. It can accept voltage ranges from 12V up to 40V. You can also power the device via the UART connection mentioned above. |
| BOOT/LOAD | A slide switch button. When set to BOOT the system will operate normally. When set to LOAD the system enters programming mode and allows you to flash updated firmware over the UART connection. Make sure it is set to BOOT afterwards for it to operate. |
| SPKR-R | Right Speaker output from the onboard stereo amplifier. Capable of powering either a 4Ω 2.5W or 8Ω 1.25W speaker at 5V. |
| SPKR-L | Left Speaker output from the onboard stereo amplifier. Capable of powering either a 4Ω 2.5W or 8Ω 1.25W speaker at 5V. |
| AUX | ****Not labelled**** The onboard stereo auxiliary headphone jack outputs stereo sound and can be fed to any type of amplifier or audio device with a 3.5mm auxiliary input jack. When a cable is inserted the system will turn off the onboard stereo amplifier and output sound from this port instead.

#### If you are using the onboard stereo amplifier with only one speaker for a mono output setup, connect the single speaker to either SPKR-R or to SPKR-L only.

#### The on-board stereo amplifier is capable of powering either a 4Ω 2.5W or 8Ω 1.25W speaker at 5V from each channel.

---
### How to use the GP Pins

Up to 20 GP pins are available on GPStar Audio XL. The GP pins have a standard 2.54mm spacing between each one. You can easily wire a standard SPDT switch or any other toggle switch, header pins or JST-XH connectors to make the connection to the GP pins.

![](images/GPStarAudioXL_GPpins.png)

| Label | Notes |
|-------|-------|
| GP01 - GP20 | There are 2 banks of pins for a total of 20. Ground the corresponding GP pin to the ground pin below it to play back a WAV file assigned to that particular GP pin.

### Optional Connectors

| Label | Pins | Notes |
|-------|------|-------|
| ST-LINK | GND/3.3V/SWCLK/SWDIO | ****Not Labelled**** These 4 pins grouped together can be connected to a ST-LINK for debugging or as a alternative way to flash updated firmware. |
| TX2/RX2 | TX2/RX2 | Alternative serial communication port used for debugging. |
| TEST |  | A button located on the lower right corner of the board. Pressing this button will play the first track on the micro SD card. |

## Configuration file and syntax usage
A configuration file is not required. However you can configure many settings by placing a GPStarAudio.ini configuration file into the root of the Micro SD Card. Many options can be set, such as the default volume, serial communication baud rate, LED settings and to how customising the GP action pins for how they handle playback.

A example file can be found in this repository. When no configuration file is present on the Micro SD Card, then GPStar Audio XL will revert to all the standard default settings.

The syntax of the file is very simple which use enclosed square brackets to setup configuration areas. 
Example syntax is as follows:

**[gpstarconfig]:** This area can set the general settings of the board. If none of these settings exist in the ini file, then the system will default to the default settings for each option.

**volume&#95;amplifier:** This sets the master gain (in dB) of the stereo speaker amplifier. The range is `-53` (quietest) to `24` (loudest). The default is `0`.

**volume&#95;aux:** This sets the master gain (in dB) of the stereo headphone amplifier. The range is `-59` (quietest) to `18` (loudest).  The default is `0`.

**serial&#95;baud&#95;rate:** You can configure the speed of the serial communication used for controlling the GPStar Audio. The default is `57600`.

**led&#95;off:** Control whether the onboard status LED flashes or not during playback or non playback. Valid values are 0 for allowing the LED to function or `1` for disabling the LED. The default is `0`.

## GP pin configuration and usage
Unique to GPStar Audio XL are the GP pins. These can be setup so you can play sounds without any coding. This area contains playback setup and configuration of the 20 GP pins. The pins are identified by gp1_ prefix in the ini file. The range is from gp1_ up to gp20_. Please refer to the example configuration file located in this repository for usage.

Note that these do not need to be configured for the GP pins to operate and if `GPStarAudio.ini` is not on the micro SD card the default settings are loaded for the GP pins.

**[gpstarmode]** Valid settings for this is `normal`, or `repeat`. Normal plays the track and it stops when it finishes. Repeat will make the track loop forever unless re activated again to do something else.

**[gpstartrigger]** Control how the playback is assigned to this GP pin. Valid settings are `press` or `hold`. Pressing will activate the GP pin and play the track. Hold will also activate any setting assigned to the GP pin for the track, however releasing will make it perform an associated action setup with **[gpstarplayback]**. Default is `normal`.

**[gpstarplayback]** This sets what the GP pin action will do if it is already playing a track associated with it. Valid settings are `stop`, `pause`, `restart`. When pause is used, activating the GP pin again will resume the playback of the track where it left off.

**[gpstartracks]** The track number on the MicroSD Card. The default is the pin number of this GP pin. For example, set this to 100 to play the wav file with the 100_ prefix.

**[gpstartrackvolume]** The default volume for the individual track playback. Valid ranges are 0 to -59, with 0 being the loudest and -59 being the most quiet. The default is 0.

**[gpstarfade]** Control if a track does any fading. Valid settings are `nofade`, `fadeout`.

**[gpstarfadeduration]** Set the duration in seconds of the track fade if enabled. The default is 0.

## Preparing your audio files
GPStar Audio plays audio from uncompressed WAV files from a MicroSD Card of any size formatted as FAT32. 
Please note that MicroSD cards larger than 32GB often come formatted in exFAT. Please reformat these cards as FAT32 using 32KB block size. Mac and Linux users can format normally with their standard disk utility software. Windows users will need to use the FAT32 Format utility found on the gpstartechnologies.com support and downloads page to format cards with capacities larger than 32GB. 

The currently supported WAV file format is 16 bit at 44.1kHz stereo. Other audio formats can be easily converted with any audio editor or free ones. All WAV files must be loaded onto the root of the MicroSD card and named appropriately with a prefixed number sequence of `###_filename.wav`. For example, `001_sound1.wav, 002_sound2.wav, 4000_sound3.wav`. The most and only important thing to remember in the naming is the `###_` prefix for all tracks. 

You must also **remove** any metadata that may be present in the WAV files. This can be done in many free audio editors such as Audacity.

**It is highly recommended to use high quality and fast MicroSD cards, otherwise audio playback can be slow or delayed. We recommend cards as fast or faster than Sandisk Extreme A1/A2 U3 V30.**

## Links

* [GPStar Audio product page](https://gpstartechnologies.com/products/gpstar-audio)
* [GPStar Audio XL product page](https://gpstartechnologies.com/products/gpstar-audio-xl)
* [GPStar Audio firmware downloads](https://gpstartechnologies.com/pages/support-downloads)


## License

Released under the GNU GPLv3 license.

Copyright (c) 2024 GPStar Technologies.
