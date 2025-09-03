using Godot;

class LowPassFilter
{
	private float lastSample = 0f;
	public float alpha;

	private float sampleRate;

	public LowPassFilter(float cutoffFreq, float sampleRate)
	{
		this.sampleRate = sampleRate;
		alpha = getAlpha(cutoffFreq);
	}

	public void setCutOff(float cutOffFreq)
	{
		alpha = getAlpha(cutOffFreq);
	}

	public float getAlpha(float cutoffFreq)
	{
		return (Mathf.Tau * cutoffFreq) / (this.sampleRate + Mathf.Tau * cutoffFreq);
	}

	public float filter(float sample)
	{
		float nextSample = lastSample + alpha * (sample - lastSample);
		lastSample = nextSample;
		return nextSample;
	}
}