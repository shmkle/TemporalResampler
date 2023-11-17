using OpenTabletDriver.Plugin;
using OpenTabletDriver.Plugin.Attributes;
using OpenTabletDriver.Plugin.Output;
using OpenTabletDriver.Plugin.Tablet;
using OpenTabletDriver.Plugin.Timing;
using System.Numerics;
using System.Runtime.Intrinsics.X86;


[PluginName("Temporal Resampler"), DeviceHub()]
public class TemporalResampler : AsyncPositionedPipelineElement<IDeviceReport>
{
    public TemporalResampler() : base()
    {
    }

    public override PipelinePosition Position => PipelinePosition.PostTransform;

    [Property("Frame Time Shift"), DefaultPropertyValue(0.5f), ToolTip
(
    "Default: 0.5\n\n" +
    "Shifts the time to add or remove predicted points. Recommended values: 0.0 - 1.0, any values beyond this would be counterintuitive and buggy.\n" +
    "0.0 == 0% predicted, one frame of latency, beautiful lines\n" +
    "0.5 == 50% predicted, half frame of latency, reasonable lines\n" +
    "1.0 == 100% predicted, no latency, ugly lines. it works well if you have any smoothing"
)]
    public float frameShift { set; get; }

    [Property("EMA Weight"), DefaultPropertyValue(1.0f), ToolTip
    (
        "Default: 1.0\n\n" +
        "Adds or removes smoothing. Range: 0.0 - 1.0\n" +
        "1.0 == no effect\n" +
        "lower == adds more, or removes more smoothing"
    )]
    public float weight { set; get; }

    [Property("Reverse EMA"), DefaultPropertyValue(false), ToolTip
    (
        "Default: false\n\n" +
        "Determines if EMA smoothing is added or removed\n" +
        "false == uses EMA smoothing. adds latency, smooth cursor\n" +
        "true == uses reverse EMA smoothing. can reverses hardware smoothing, snappy cursor"
    )]
    public bool reverseSmoothing { set; get; }

    protected override void UpdateState()
    {
        if (State is ITabletReport report && PenIsInRange())
        {
            t += rpsAvg / Frequency * m;
            report.Position = Trajectory(MathF.Min(MathF.Max(t, -2.0f), 2.0f) + 2.0f, p3, p2, p1);
            report.Pressure = pressure;
            State = report;
            OnEmit();
        }
    }
    protected override void ConsumeState()
    {
        if (State is ITabletReport report)
        {
            var consumeDelta = (float)reportStopwatch.Restart().TotalSeconds;
            pressure = report.Pressure;
            Vector2 position = report.Position;
            Vector2 p0;

            if (consumeDelta < 0.15f && consumeDelta > 0.0f)
                rpsAvg += (1.0f / consumeDelta - rpsAvg) * 0.01f;

            if (reverseSmoothing)
            {
                p0 = lp + (position - lp) / weight;
                lp = position;
            }
            else
            {
                p0 = p1 + (position - p1) * weight;
            }

            p3 = p2;
            p2 = p1;
            p1 = p0;

            if (t > -5.0f & t < 5.0f)
            {
                t -= 1.0f;
                m += (frameShift - t - m) * 0.5f;
            }
            else
            {
                t = frameShift - 1.0f;
                m = 1.0f;
            }
        }
        else OnEmit();
    }

    private static Vector2 Trajectory(float t, Vector2 v3, Vector2 v2, Vector2 v1)
    {
        return v3 + (2.0f * v2 - v3 - (v1 + v3) * 0.5f) * t + ((v1 + v3) * 0.5f - v2) * MathF.Pow(t, 2.0f);
    }

    private Vector2 p3, p2, p1, lp;
    private uint pressure;
    private float t = -0.5f;
    private float m = 1.0f;
    private float rpsAvg = 200.0f;
    private HPETDeltaStopwatch reportStopwatch = new HPETDeltaStopwatch();
}
