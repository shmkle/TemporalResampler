# TemporalResampler Plugin

[![](https://img.shields.io/github/downloads/shmkle/TemporalResampler/total.svg)](https://github.com/shmkle/TemporalResampler/releases/latest)

*Meant for [OpenTabletDriver](https://github.com/OpenTabletDriver/OpenTabletDriver)*


### The purpose:
Improves cursor clarity for higher refresh rate monitors with minimal to zero latency. It is very effective if your tablet Reports at 133rps (ctl-472, ctl-672) and your monitor Updates at 240hz.

### The Values:

**Frame Time Shift:**
    
    Shifts the time to add or remove predicted points. Default: 0.5, Recommended values: 0.0 - 1.0, any values beyond this would be counterintuitive and buggy
    0.0 == 0% predicted, one frame of latency, beautiful lines
    0.5 == 50% predicted, half frame of latency, reasonable lines
    1.0 == 100% predicted, no latency, ugly lines. it works well if you have any smoothing
    
**EMA Weight:** 
   
    Adds or removes smoothing. Default: 1.0, Range: 0.0 - 1.0
    1.0 == no effect
    lower == adds more or removes more smoothing
    
**Reverse EMA:**   

    Determines if EMA smoothing is added or removed. Default: false
    false == uses EMA smoothing. adds latency, smooth cursor
    true == uses reverse EMA smoothing. can reverse hardware smoothing, snappy cursor. follow the guidelines given in the [Recontructer](https://github.com/X9VoiD/VoiDPlugins/wiki/Reconstructor) wiki to find the appropriate value
