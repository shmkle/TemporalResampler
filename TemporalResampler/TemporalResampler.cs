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

            if (chatterDiameter > 0)
            {
                float t1 = t / rpsAvg;
                float t2 = (t + 1.0f) / rpsAvg;

                if (t > 0.0f)
                    output = pos1 + t1 * vel1 + 0.5f * t1 * t1 * accel1;
                else
                    output = pos2 + t2 * vel2 + 0.5f * t2 * t2 * accel2;
            }
            else
                output = Trajectory(MathF.Min(MathF.Max(t, -2.0f), 2.0f) + 2.0f, p3, p2, p1);

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
            float consumeDelta = (float)reportStopwatch.Restart().TotalSeconds;
            Vector2 position = report.Position;
            Vector2 p0;
            pressure = report.Pressure;

            if (consumeDelta < 0.02f && consumeDelta > 0.0f)
                rpsAvg += (1.0f / consumeDelta - rpsAvg) * 0.01f;
            else
                placeDebounce = false;

            if (placeDebounce)
            {
                float chatterStrength = chatterDiameter;

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

                if (t > -3.0f & t < 3.0f)
                {
                    t -= 1.0f;

                    if (chatterDiameter > 0)
                    {
                        float newDelta = 1.0f / rpsAvg;

                        m += (frameShiftBase - t - m) * 0.33f;

                        pos2 = pos1;
                        vel2 = vel1;
                        accel2 = accel1;

                        pos1 = pos2 + newDelta * vel2 + 0.5f * newDelta * newDelta * accel2;
                        vel1 = vel2 + newDelta * accel2;

                        Vector2 correctVector = Trajectory(1.0f + frameShift, p3, p2, p1) - pos1;
                        float correctPower = MathF.Min(MathF.Max(correctVector.Length() / chatterStrength - 1.0f, 0.0f), 1.0f);
                        Vector2 correctAccel = (correctVector - vel1 * newDelta) / (newDelta * newDelta);

                        Vector2 toVector = Vector2.Lerp(p1 - p2, Trajectory(3.0f, p3, p2, p1) - p1, frameShift) ;
                        float toPower = MathF.Min(MathF.Max(toVector.Length() / chatterStrength - 1.0f, 0.0f), 1.0f);
                        toVector = toVector * toPower;
                        Vector2 toAccel = (toVector - vel1 * newDelta) / (newDelta * newDelta);

                        accel1 = Vector2.Lerp(toAccel, correctAccel, correctPower);
                    }
                    else
                        m += (frameShift - t - m) * 0.33f;
                }
                else
                    resetValues(position);
            }
            else
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
        
        m = 1.0f;
        if (chatterDiameter != 0)
            t = frameShiftBase - 1.0f;
        else
            t = frameShift - 1.0f;
    }

    private static Vector2 Trajectory(float t, Vector2 v3, Vector2 v2, Vector2 v1)
    {
        return v3 + (2.0f * v2 - v3 - (v1 + v3) * 0.5f) * t + ((v1 + v3) * 0.5f - v2) * t * t;
    }

    private Vector2 p3, p2, p1, lp;
    private Vector2 pos1, vel1, accel1;
    private Vector2 pos2, vel2, accel2;
    private bool placeDebounce = false;
    private uint pressure;
    private float t = -0.5f;
    private float m = 1.0f;
    private float rpsAvg = 200.0f;
    private float frameShiftBase = 0.90f;
    private HPETDeltaStopwatch reportStopwatch = new HPETDeltaStopwatch();
    private HPETDeltaStopwatch updateStopwatch = new HPETDeltaStopwatch();
}
