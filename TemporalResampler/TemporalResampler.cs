using OpenTabletDriver.Plugin;
using OpenTabletDriver.Plugin.Attributes;
using OpenTabletDriver.Plugin.Output;
using OpenTabletDriver.Plugin.Tablet;
using OpenTabletDriver.Plugin.Timing;
using System.Numerics;


[PluginName("Temporal Resampler"), DeviceHub()]
public class TemporalResampler : AsyncPositionedPipelineElement<IDeviceReport>
{
    public TemporalResampler() : base()
    {
    }

    public override PipelinePosition Position => PipelinePosition.Raw;

    [Property("Frame Time Shift"), DefaultPropertyValue(0.5f), ToolTip
    (
        "Default: 0.5\n\n" +
        "Shifts the time to add or remove predicted points. Range: 0.0 - 1.0\n" +
        "0.0 == 0% predicted, one frame of latency, beautiful lines\n" +
        "0.5 == 50% predicted, half frame of latency, reasonable lines\n" +
        "1.0 == 100% predicted, no latency, ugly lines. it works well if you have any smoothing"
    )]
    public float frameShift 
    {
        set => _frameShift = Math.Clamp(value, 0, 1);
        get => _frameShift;
    }
    public float _frameShift;

    [Property("Frame Time Sync"), DefaultPropertyValue(0.25f), ToolTip
    (
        "Default: 0.25\n\n" +
        "Change this if your cursor twitches out of line, value controls how strong the filter wants to stay in sync. Range: 0.0 - 1.0\n" +
        "higher == worse looking cursor gaps, less glitching\n" +
        "lower = better-looking cursor gaps, more glitching"
    )]
    public float syncPower
    {
        set => _syncPower = Math.Clamp(value, 0, 1);
        get => _syncPower;
    }
    public float _syncPower;

    [Property("EMA Weight"), DefaultPropertyValue(1f), ToolTip
    (
        "Default: 1.0\n\n" +
        "Adds or removes smoothing. Range: 0.0 - 1.0\n" +
        "1.0 == no effect\n" +
        "lower == adds more, or removes more smoothing"
    )]
    public float weight
    {
        set => _weight = Math.Clamp(value, 0, 1);
        get => _weight;
    }
    public float _weight;

    [Property("Reverse EMA"), DefaultPropertyValue(false), ToolTip
    (
        "Default: false\n\n" +
        "Determines if EMA smoothing is added or removed.\n" +
        "false == uses EMA smoothing. adds latency, smooth cursor\n" +
        "true == uses reverse EMA smoothing. can reverses hardware smoothing, snappy cursor"
    )]
    public bool reverseSmoothing { set; get; }

    [Property("Chatter Diameter"), DefaultPropertyValue(0), ToolTip
    (
        "Default: 0\n\n" +
        "Diameter of unusable chatter information in tablet coordinates.\n" +
        "0 == off. will make the filter use a different process for extrapolating inputs\n" +
        "higher == on. increase this value until holding your pen in place does not chatter"
    )]
    public int chatterDiameter { set; get; }

protected override void UpdateState()
    {
        if (State is ITabletReport report && PenIsInRange())
        {
            float updateDelta = (float)updateStopwatch.Restart().TotalSeconds;
            t += rpsAvg * updateDelta * m * 0.5f;
            Vector2 output;
            float limit0 = frameShift + 0.5f;
            float limit1 = frameShift - 1.5f;

            if (chatterDiameter > 0) // chatter diameter active
            {
                float t1 = Math.Min(t, limit0) / rpsAvg;
                float t2 = Math.Max(t + 1f, limit1) / rpsAvg;

                if (t > 0f) // ahead of latest report
                    output = pos1 + t1 * vel1 + 0.5f * t1 * t1 * accel1;
                else // behind latest report
                    output = pos2 + t2 * vel2 + 0.5f * t2 * t2 * accel2;

            }
            else // chatter diameter not active
                output = Trajectory(Math.Clamp(t, limit1, limit0) + 2f, p3, p2, p1);

            t += rpsAvg * updateDelta * m * 0.5f;
            report.Position = output;
            report.Pressure = pressure;
            State = report;
            OnEmit();
        }
    }
    protected override void ConsumeState()
    {
        if (State is ITabletReport report)
        {
            Log.Debug("frameShift", frameShift.ToString());
            float consumeDelta = (float)reportStopwatch.Restart().TotalSeconds;
            Vector2 position = report.Position;
            Vector2 p0;
            pressure = report.Pressure;

            if (consumeDelta < 0.03f && consumeDelta > 0f) // detects if the tablet was out of range
                rpsAvg += (1f / consumeDelta - rpsAvg) * 0.01f;
            else
                placeDebounce = false;

            if (placeDebounce) // 2nd+ frames since detection
            {
                float chatterStrength = chatterDiameter * 0.5f;

                if (reverseSmoothing)
                {
                    p0 = lp + (position - lp) / weight;
                    lp = position;
                    chatterStrength /= weight;
                }
                else
                {
                    p0 = p1 + (position - p1) * weight;
                    chatterStrength *= weight;
                }

                p3 = p2;
                p2 = p1;
                p1 = p0;

                if (t > -3f & t < 3f) // fixes tabbing into programs
                {
                    t -= 1f;

                    if (chatterDiameter > 0) // chatter diameter active
                    {
                        float newDelta = 1f / rpsAvg;
                        float edgeSharpness = 1.414f;

                        m += (frameShiftBase - t - m) * syncPower;

                        pos2 = pos1;
                        vel2 = vel1;
                        accel2 = accel1;

                        pos1 = pos2 + newDelta * vel2 + 0.5f * newDelta * newDelta * accel2;
                        vel1 = vel2 + newDelta * accel2;

                        Vector2 traject1 = Trajectory(2f + frameShift, p3, p2, p1);
                        Vector2 traject2 = Trajectory(1f + frameShift, p3, p2, p1);
                        float mix = Math.Clamp(Vector2.Distance(pos1, traject2) / chatterStrength * edgeSharpness, 0f, 1f);

                        Vector2 accelVector = traject1 - pos1 * mix - traject2 * (1f - mix);
                        accelVector *= Math.Clamp((accelVector.Length() / chatterStrength - 1f) * edgeSharpness, 0f, 1f);

                        accel1 = (accelVector - vel1 * newDelta) / (newDelta * newDelta);
                    }
                    else // chatter diameter not active
                        m += (frameShift - t - m) * syncPower;

                }
                else
                    resetValues(position);

            }
            else // 1st frame since detection
            {
                resetValues(position);
                placeDebounce = true;
            }

        }
        else OnEmit();
    }
    private void resetValues(Vector2 p0)
    {
        lp = p0;
        p1 = p0;
        p2 = p0;
        p3 = p0;
        pos1 = p0;
        pos2 = p0;
        vel1 = Vector2.Zero;
        vel2 = Vector2.Zero;
        accel1 = Vector2.Zero;
        accel2 = Vector2.Zero;
        m = 1f;

        if (chatterDiameter != 0)
            t = frameShiftBase - 1f;
        else
            t = frameShift - 1f;
    }

    private static Vector2 Trajectory(float t, Vector2 v3, Vector2 v2, Vector2 v1) // cool formula for finding the best-fit quadratic function that passes 3 points
    {
        return v3 + (2f * v2 - v3 - (v1 + v3) * 0.5f) * t + ((v1 + v3) * 0.5f - v2) * t * t;
    }

    // values
    private Vector2 p3, p2, p1, lp;
    private Vector2 pos1, vel1, accel1;
    private Vector2 pos2, vel2, accel2;
    private bool placeDebounce = false;
    private uint pressure;
    private float t = -0.5f;
    private float m = 1f;
    private float rpsAvg = 200f;
    private float frameShiftBase = 0.90f;
    private HPETDeltaStopwatch reportStopwatch = new HPETDeltaStopwatch();
    private HPETDeltaStopwatch updateStopwatch = new HPETDeltaStopwatch();
}
