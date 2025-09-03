using System;
using Godot;

class DelayLine
{
	private RingBuffer buffer;
	public float maxLengthSeconds;

	private int sampleRate;

	public DelayLine(float lengthSeconds, int sampleRate)
	{
		maxLengthSeconds = lengthSeconds;
		this.sampleRate = sampleRate;
		int bufferLength = (int)Math.Ceiling(lengthSeconds * sampleRate);
		buffer = new RingBuffer(bufferLength);
	}

	public void Push(float sample)
	{
		buffer.Write(sample);
		buffer.advance();
	}

	public float Pop(float effectiveLengthSeconds = -1)
	{
		if (effectiveLengthSeconds == -1) {
			effectiveLengthSeconds = maxLengthSeconds;
		}

		int samplesBack = (int)Mathf.Ceil(effectiveLengthSeconds * sampleRate);
		return buffer.Read(samplesBack);
	}
}