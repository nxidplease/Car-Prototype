using Godot;
// VQ35DE	3.5 L (3,498 cc)	Bore: 95.5 mm Stroke: 81.4 mm


struct CylinderOut
{
	public float intakeOut;
	public float vibrationsOut;

	public float exhaust;

	public CylinderOut(float intakeOut, float vibrationsOut, float exhaust)
	{
		this.intakeOut = intakeOut;
		this.vibrationsOut = vibrationsOut;
		this.exhaust = exhaust;
	}
}

class Cylinder
{

	// Speed of sound in Air at 260C(the average temp inside a cylinder)
	private const float speedOfSoundInCylinder = 487;

	private const float maxVolume = 583.0704466f;

	// Extended divided by fully compressed
	private const float compressionRatio = 10f;

	// The clearing left when fully compressed
	private const float clearVolume = maxVolume / compressionRatio;

	private const float sweptVolume = maxVolume - clearVolume;


	// Fire time in cycles (0-1)
	private float firingTime;

	public float lastOutput;

	private float pistonPos = 0;

	/// <summary>
	/// Second chamber output is split into half to intake and exauhst
	/// </summary>
	public WaveGuide cylinderChamber;

	/// <summary>
	/// Alpha is free end, beta is cylinder end
	/// </summary>
	public WaveGuide intakeCollector;

	/// <summary>
	/// First chamber out is to straight pipe end, second chamber is cylinder end
	/// </summary>
	public WaveGuide extractor;

	private int index;

	public Cylinder(int sampleRate, float firingTime, int index)
	{
		this.firingTime = firingTime;

		// Delay time of a VQ35DE cylinder: 1.67 * 10^-4, this is one way trip
		cylinderChamber = new WaveGuide(1.67e-4f, -1, .7f, sampleRate);

		// The delay is taken from https://github.com/DasEtwas/enginesound/blob/master/src/default.esc
		intakeCollector = new WaveGuide(1.458e-4f, 0, -0.5f, sampleRate);
		extractor = new WaveGuide(1.458e-4f, 0, 0.1f, sampleRate);

		this.index = index;
	}

	public CylinderOut Write(float crankPos, float throttle, float intakeNoise, float prevStraightPipe)
	{
		pistonPos = PistonMotion(crankPos);
		float cylinderIn = FuelIgnition(crankPos, firingTime) * throttle + pistonPos;
		lastOutput = cylinderIn;

		// 0 - Fully down, 1- Fully compressed
		float normalizedCrankPos = (pistonPos + 1f) * .5f;
		float currVolume = clearVolume + sweptVolume * normalizedCrankPos;
		float normalizedCylinderVolume = currVolume / maxVolume;

		float maxDelay = cylinderChamber.firstChamber.maxLengthSeconds;

		WaveGuideOutput cylinderWgOut = cylinderChamber.Pop(maxDelay * normalizedCylinderVolume);

		WaveGuideOutput intakeWgOut = intakeCollector.Pop();
		WaveGuideOutput exhaustWgOut = extractor.Pop();

		float intakeOut = intakeWgOut.firstChamberOut; // Free end
		float cylOut = cylinderWgOut.secondChamberOut;

		float inValve = intakeValve(crankPos); // 1 is fully open, 0 is closed
		float exValve = exhaustValve(crankPos); // 1 is fully open, 0 is closed

		intakeCollector.alpha = (.7f - .4f * inValve);
		extractor.beta = (.7f - .4f * exValve);

		cylinderChamber.alpha = Mathf.Max(.7f - .4f * inValve, .7f - .4f * exValve); // Since the valves are at the same "end" of the cylinder they should affect the same coefficient
		// cylinderChamber.alpha = 1 - inValve;
		// cylinderChamber.beta = 1 - exValve;

		float toIntake = cylOut * inValve;
		float toExhaust = cylOut * exValve;
		// float toIntake = cylOut * .5f * (1 - inValve);
		// float toExhaust = cylOut * .5f * (1 - exValve);
		
		// intakeCollector.Push(0, toIntake + intakeNoise * inValve);
		intakeCollector.Push(intakeNoise * inValve + toIntake, 0);

		// GD.Print($"Cylinder {index} toExhaust: {toExhaust}");

		extractor.Push(toExhaust, prevStraightPipe);

		// float fromIntakeToCylinder = intakeWgOut.secondChamberOut * inValve;
		// float fromExhaust = exhaustWgOut.secondChamberOut * exValve;
		float fromIntakeToCylinder = intakeWgOut.secondChamberOut;
		float fromExhaust = exhaustWgOut.secondChamberOut;

		cylinderChamber.Push(fromIntakeToCylinder + fromExhaust, cylinderIn);

		return new CylinderOut(intakeOut, lastOutput, exhaustWgOut.firstChamberOut);
	}

	private float exhaustValve(float crankPos)
	{
		if (crankPos < .75f || crankPos > 1)
		{
			return 0;
		}

		return -Mathf.Sin(2 * Mathf.Tau * crankPos);
	}

	private float intakeValve(float crankPos)
	{
		if (crankPos < 0 || crankPos > .25f)
		{
			return 0;
		}

		return Mathf.Sin(2 * Mathf.Tau * crankPos);
	}

	private float PistonMotion(float crankPos)
	{
		return Mathf.Cos(2 * Mathf.Tau * crankPos);
	}

	// t - Time(relative to full cycle) needed by the fuel to explode
	private float FuelIgnition(float crankPos, float t)
	{
		if (crankPos < 0 || crankPos > t)
		{
			return 0;
		}

		return Mathf.Sin(Mathf.Tau * (crankPos * t + .5f));
	}
}