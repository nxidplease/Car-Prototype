using Godot;
// VQ35DE	3.5 L (3,498 cc)	Bore: 95.5 mm Stroke: 81.4 mm


struct CylinderOut
{
	public float intakeOut;
	public float vibrationsOut;

	public CylinderOut(float intakeOut, float vibrationsOut)
	{
		this.intakeOut = intakeOut;
		this.vibrationsOut = vibrationsOut;
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

	// Alpha is split into half to intake and exauhst
	public WaveGuide cylinderChamber;

	// Alpha is free end, beta is cylinder end
	public WaveGuide intakeCollector;

	// Alpha is free end, beta is cylinder end
	public WaveGuide exhaustCollector;

	public Cylinder(int sampleRate, float firingTime)
	{
		this.firingTime = firingTime;

		// Delay time of a VQ35DE cylinder: 1.67 * 10^-4, this is one way trip
		cylinderChamber = new WaveGuide(1.67e-4f, -1, 1, sampleRate);

		// The delay is taken from https://github.com/DasEtwas/enginesound/blob/master/src/default.esc
		intakeCollector = new WaveGuide(1.458e-4f, -0.5f, 0, sampleRate);
		exhaustCollector = new WaveGuide(1.458e-4f, 0.1f, 0, sampleRate);
	}

	public CylinderOut Write(float crankPos, float throttle, float intakeNoise)
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
		WaveGuideOutput exhaustWgOut = exhaustCollector.Pop();

		float intakeOut = intakeWgOut.firstChamberOut; // Free end
		float cylOut = cylinderWgOut.firstChamberOut;

		float inValve = intakeValve(crankPos); // 1 is fully open, 0 is closed
		float exValve = exhaustValve(crankPos); // 1 is fully open, 0 is closed

		intakeCollector.beta = (1 - inValve);
		exhaustCollector.beta = (1 - exValve);

		cylinderChamber.alpha = Mathf.Max(1 - inValve, 1 - exValve); // Since the valves are at the same "end" of the cylinder they should affect the same coefficient
																																 // cylinderChamber.beta = 1 - exValve;

		float toIntake = cylOut * .5f * (1 - inValve);
		float toExhaust = cylOut * .5f * (1 - exValve);
		intakeCollector.Push(0, toIntake + intakeNoise);
		exhaustCollector.Push(0, toExhaust);

		float fromIntakeToCylinder = intakeWgOut.secondChamberOut * inValve;
		float fromExhaust = exhaustWgOut.secondChamberOut * exValve;

		cylinderChamber.Push(fromIntakeToCylinder + fromExhaust + cylinderIn, 0);

		return new CylinderOut(intakeOut, lastOutput);
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