# TemporalResampler Plugin

[![](https://img.shields.io/github/downloads/shmkle/TemporalResampler/total.svg)](https://github.com/shmkle/TemporalResampler/releases/latest)

*Meant for [OpenTabletDriver](https://github.com/OpenTabletDriver/OpenTabletDriver)*


### The purpose:
Improves cursor clarity for higher refresh rate monitors with minimal latency. It is very effective if your tablet reports at 133rps (ctl-472, ctl-672) and your monitor updates at 240hz.

### The Values:

**Frame Time Shift:**
    
    Default: 0.5
    Range: 0.0 - 1.0

    Shifts the time to add or remove predicted points.
    0.0 == 0% predicted, one frame of latency, beautiful lines
    0.5 == 50% predicted, half frame of latency, reasonable lines
    1.0 == 100% predicted, no latency, ugly lines. it works well if you have any smoothing


**Chatter Diameter:** 

    Default: 0
    
    Diameter of unusable chatter information in tablet coordinates.
    0 == off. will make the filter use a different process for extrapolating inputs
    higher == on. increase this value until holding your pen in place does not chatter


**Smoothing Latency:** 

    Default: 0ms

    The amount of latency added by smoothing, approach is similar to Hawku smoothing.

    
**Reverse EMA:** 

    Default: 1
    Range: 0.0 - 1.0
    
    Removes hardware smoothing, fine-tuned this to your tablet. Follow the guide given in the Reconstructor filter wiki (linked below). 
    1.0 == no effect
    lower == removes more hardware smoothing


[Reconstructor](https://github.com/X9VoiD/VoiDPlugins/wiki/Reconstructor)
