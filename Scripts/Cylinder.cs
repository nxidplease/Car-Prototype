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

	private float prevCylinderRevOut;

	private float prevExtractorRevOut;

	public Cylinder(int sampleRate, float firingTime, int index)
	{
		this.firingTime = firingTime;

		// Delay time of a VQ35DE cylinder: 2.39 * 10^-4, this is one way trip using 340.29 m/s for speed of sound
		cylinderChamber = new WaveGuide(2.39e-4f, -1, .7f, sampleRate);

		// The delay is taken from https://github.com/DasEtwas/enginesound/blob/master/src/default.esc
		intakeCollector = new WaveGuide(1.458e-4f, 0, -0.5f, sampleRate);
		extractor = new WaveGuide(1.458e-4f, 0, 0.1f, sampleRate);

		this.index = index;
	}

	public CylinderOut Write(float crankPos, float throttle, float intakeNoise, float prevStraightPipe)
	{
		pistonPos = PistonMotion(crankPos);
		float cylinderIn = FuelIgnition(crankPos, firingTime) * throttle + pistonPos;
		float inValve = intakeValve(crankPos); // 1 is fully open, 0 is closed
		float exValve = exhaustValve(crankPos); // 1 is fully open, 0 is closed

		float intakeCoef = .9f - .8f * inValve;
		float exhaustCoef = .9f - .8f * exValve;

		intakeCollector.fwdOutRefl = intakeCoef;
		cylinderChamber.revOutRefl = intakeCoef;
		cylinderChamber.fwdOutRefl = exhaustCoef;
		extractor.revOutRefl = exhaustCoef;

		// 0 - Fully down, 1- Fully compressed
		float normalizedCrankPos = (pistonPos + 1f) * .5f;
		float currVolume = clearVolume + sweptVolume * normalizedCrankPos;
		float normalizedCylinderVolume = currVolume / maxVolume;

		float maxDelay = cylinderChamber.fwdChamber.maxLengthSeconds;


		// intakes
		intakeCollector.Push(inValve * intakeNoise, prevCylinderRevOut);
		WaveGuideOutput intakeWgOut = intakeCollector.Pop();
		float intakeOut = intakeWgOut.revChamberOut; // Free end

		// cylinders
		cylinderChamber.Push(cylinderIn + intakeWgOut.fwdChamberOut, prevExtractorRevOut);
		WaveGuideOutput cylinderWgOut = cylinderChamber.Pop(maxDelay * normalizedCylinderVolume);
		prevCylinderRevOut = cylinderWgOut.revChamberOut;
		lastOutput = cylinderIn + inValve + exValve;
		
		// extractor
		extractor.Push(cylinderWgOut.fwdChamberOut, prevStraightPipe);
		WaveGuideOutput exhaustWgOut = extractor.Pop();
		float extractorFwdOut = exhaustWgOut.fwdChamberOut;
		prevExtractorRevOut = exhaustWgOut.revChamberOut;

		return new CylinderOut(intakeOut, lastOutput, extractorFwdOut);
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

		// return Mathf.Sin(Mathf.Tau * (crankPos * t + .5f));
		return Mathf.Sin(Mathf.Tau * crankPos / t);
	}
}