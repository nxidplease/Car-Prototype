using Godot;

class RingBuffer
{
	private int dataCount = 0;
	private Vector2[] buffer;

	private int writeIndex = 0;
	private int readIndex = 0;

	public RingBuffer(int capacity)
	{
		buffer = new Vector2[capacity];
	}

	public bool isEmpty()
	{
		return dataCount == 0;
	}

	public bool isFull()
	{
		return dataCount == buffer.Length;
	}

	public int availableData()
	{
		return dataCount;
	}

	public int availableSpace()
	{
		return buffer.Length - dataCount;
	}

	public void write(Vector2 item)
	{
		buffer[writeIndex] = item;
		writeIndex = (writeIndex + 1) % buffer.Length;
		dataCount++;
	}

	public Vector2 read()
	{
		Vector2 readValue = buffer[readIndex];
		readIndex = (readIndex + 1) % buffer.Length;
		dataCount--;

		return readValue;
	}
}