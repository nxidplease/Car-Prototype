using Godot;

struct WaveGuideOutput {

	public WaveGuideOutput(float fwdChamberOut, float revChamberOut)
	{
		this.fwdChamberOut = fwdChamberOut;
		this.revChamberOut = revChamberOut;
	}

	public float fwdChamberOut;
	public float revChamberOut;
}

/// <summary>
/// WaveGuide represents a "tube" of an arbitrary length(controled by the delay time),
/// 
/// first_in and secondChamberOut represent one end of the tube, while second_in and firstChamberOut
/// represent the other.
///
/// </summary>
class WaveGuide
{
	const float MAX_WAVE_GUIDE_AMP = 20f;
	public DelayLine fwdChamber;
	public DelayLine revChamber;

	// reflection factor for the first chamber
	public float revOutRefl;

	// reflection factor for the second chamber
	public float fwdOutRefl;

	float prevFwdChamberOut;

	float prevRevChamberOut;

	public WaveGuide(float delaySeconds, float revOutRefl, float fwdOutRefl, int sampleRate)
	{
		fwdChamber = new DelayLine(delaySeconds, sampleRate);
		revChamber = new DelayLine(delaySeconds, sampleRate);
		this.revOutRefl = revOutRefl;
		this.fwdOutRefl = fwdOutRefl;
	}

	public WaveGuideOutput Pop(float effectiveLengthSeconds = -1)
	{
		prevFwdChamberOut = Dampen(fwdChamber.Pop(effectiveLengthSeconds));
		prevRevChamberOut = Dampen(revChamber.Pop(effectiveLengthSeconds));

		return new WaveGuideOutput(prevFwdChamberOut * (1 - Mathf.Abs(fwdOutRefl)), prevRevChamberOut * (1 - Mathf.Abs(revOutRefl)));
	}

	private float Dampen(float sample)
	{
		if (Mathf.Abs(sample) <= MAX_WAVE_GUIDE_AMP)
		{
			return sample;
		}

		return Mathf.Sign(sample) * -1 / (sample - MAX_WAVE_GUIDE_AMP + 1) + 1 + MAX_WAVE_GUIDE_AMP;
	}

	public void Push(float fwd_in, float rev_in)
	{
		float fwdChamberIn = prevRevChamberOut * revOutRefl + fwd_in;
		float revChamberIn = prevFwdChamberOut * fwdOutRefl + rev_in;

		fwdChamber.Push(fwdChamberIn);
		revChamber.Push(revChamberIn);
	}
}