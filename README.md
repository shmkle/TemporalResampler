# TemporalResampler Plugin

[![](https://img.shields.io/github/downloads/shmkle/TemporalResampler/total.svg)](https://github.com/shmkle/TemporalResampler/releases/latest)

*Meant for [OpenTabletDriver](https://github.com/OpenTabletDriver/OpenTabletDriver)*


### The purpose:
Improves cursor clarity for higher refresh rate monitors with minimal latency. It is very effective if your tablet reports at 133rps (ctl-472, ctl-672) and your monitor updates at 240hz.

### The Values:

**Frame Time Shift:**

    Shifts the time to add or remove predicted points. Default: 0.5, Recommended values: 0.0 - 1.0, any values beyond this would be counterintuitive and buggy
    0.0 == 0% predicted, one frame of latency, beautiful lines
    0.5 == 50% predicted, half frame of latency, reasonable lines
    1.0 == 100% predicted, no latency, ugly lines. it works well if you have any smoothing


**Frame Time Sync:**

    
    Change this if your cursor twitches out of line, value controls how strong the filter wants to stay in sync. Default: 0.25, Range: 0.0 - 1.0
    higher == worse looking cursor gaps, less glitching
    lower = better-looking cursor gaps, more glitching


**EMA Weight:** 

    Adds or removes smoothing. Default: 1.0, Range: 0.0 - 1.0
    1.0 == no effect
    lower == adds more or removes more smoothing

    
**Reverse EMA:** 

    Determines if EMA smoothing is added or removed. Default: false
    false == uses EMA smoothing. adds latency, smooth cursor
    true == uses reverse EMA smoothing. can reverse hardware smoothing, snappy cursor. Follow the guide given by the Reconstructor plugin wiki to find the appropriate value (link below)

    
**Chatter Diameter:** 

    Default: 0
    Diameter of unusable chatter information in tablet coordinates.
    0 == off. will make the filter use a different process for extrapolating inputs
    higher == on. increase this value until holding your pen in place does not chatter


[Reconstructor](https://github.com/X9VoiD/VoiDPlugins/wiki/Reconstructor)
