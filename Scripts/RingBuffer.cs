using Godot;

class RingBuffer
{

	private float[] buffer;

	private int pos = 0;

	public RingBuffer(int capacity)
	{
		buffer = new float[capacity];
		buffer.Initialize();
	}

	public void Write(float item)
	{
		buffer[pos] = item;
	}

	// Read a sample from cpacity ago samples
	// samplesBack is effective buffer capacity(to allow variable buffer length)
	public float Read(int samplesBack = -1)
	{
		if (samplesBack == -1) {
			samplesBack = buffer.Length - 1;
		}

		return buffer[(pos + buffer.Length - samplesBack) % buffer.Length];
	}

	public void advance()
	{
		pos = (pos + 1) % buffer.Length;
	}
}