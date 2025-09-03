using Godot;

struct WaveGuideOutput {

	public WaveGuideOutput(float firstChamberOut, float secondChamberOut)
	{
		this.firstChamberOut = firstChamberOut;
		this.secondChamberOut = secondChamberOut;
	}

	public float firstChamberOut;
	public float secondChamberOut;
}


class WaveGuide
{
	const float MAX_WAVE_GUIDE_AMP = 20f;
	public DelayLine firstChamber;
	public DelayLine secondChamber;

	// reflection factor for the first chamber
	public float alpha;

	// reflection factor for the second chamber
	public float beta;

	float prevFirstChamberOut;

	float prevSecondChamberOut;

	public WaveGuide(float delaySeconds, float alpha, float beta, int sampleRate)
	{
		firstChamber = new DelayLine(delaySeconds, sampleRate);
		secondChamber = new DelayLine(delaySeconds, sampleRate);
		this.alpha = alpha;
		this.beta = beta;
	}

	public WaveGuideOutput Pop(float effectiveLengthSeconds = -1)
	{
		prevFirstChamberOut = Dampen(firstChamber.Pop(effectiveLengthSeconds));
		prevSecondChamberOut = Dampen(secondChamber.Pop(effectiveLengthSeconds));

		return new WaveGuideOutput(prevFirstChamberOut * (1 - Mathf.Abs(beta)), prevSecondChamberOut * (1 - Mathf.Abs(alpha)));
	}

	private float Dampen(float sample)
	{
		if (Mathf.Abs(sample) <= MAX_WAVE_GUIDE_AMP)
		{
			return sample;
		}

		return Mathf.Clamp(sample, -MAX_WAVE_GUIDE_AMP, MAX_WAVE_GUIDE_AMP);
	}

	public void Push(float first_in, float second_in)
	{
		float firstChamberIn = prevSecondChamberOut * alpha + first_in;
		float secondChamberIn = prevFirstChamberOut * beta + second_in;

		firstChamber.Push(firstChamberIn);
		secondChamber.Push(secondChamberIn);
	}
}