# TemporalResampler Plugin

[![](https://img.shields.io/github/downloads/shmkle/TemporalResampler/total.svg)](https://github.com/shmkle/TemporalResampler/releases/latest)

*Meant for [OpenTabletDriver](https://github.com/OpenTabletDriver/OpenTabletDriver)*

*[Support me here](https://www.patreon.com/Shmekle)*

### The purpose:
Improves cursor clarity for higher refresh rate monitors with minimal latency. It is very effective if your tablet reports at 133rps (ctl-472, ctl-672) and your monitor updates at 240hz.

### The Values:

**Prediction Ratio:**

    Default: 0.5
    Range: 0.0 - 1.0

    Determines the time distance the filter predicts inputs for each tablet update.
    Prediction brought to you by Kalman filtering.
    0.0 == [slower] 0% predicted, one rps of latency
    0.5 == [balanced] 50% predicted, half rps of latency
    1.0 == [overkill] 100% predicted, no added latency (works best with some smoothing)


**Follow Radius:** 

    Default: 0.0
    
    Radius of cursor trailing distance in millimeters.


**Smoothing Latency:** 

    Default: 0ms

    The amount of latency added by smoothing, approach is similar to Hawku smoothing.

    
**Reverse EMA:** 

    Default: 1
    Range: 0.0 - 1.0
    
    Removes hardware smoothing, fine-tuned this to your tablet. 
    Follow the guide given in the Reconstructor filter wiki (linked below). 
    1.0 == no effect
    lower == removes more hardware smoothing


**Maximize Frequency**

    Default: True

    Wires ConsumeState to UpdateState for a further increased update frequency with no elevated OTD CPU usage.
    Disable if any unexpected behavior is happening or you just don't want it on.
    
**Log Stats**

    Default: False

    Logs the time and physical latency in OpenTableDriver Console.
    Make sure to disable when you are done so you don't waste memory.
    [False] == logging disabled
    [True] == logging enabled, check the Console tab in OpenTableDriver

[Reconstructor](https://github.com/X9VoiD/VoiDPlugins/wiki/Reconstructor)
