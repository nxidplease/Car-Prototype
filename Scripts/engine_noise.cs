using Godot;
using System;

public class engine_noise : Node
{
	private const float DEFAULT_MIN_RPM = 1000;
	private const float DEFAULT_MAX_RPM = 8500;
	private const int sampleRate = 44100;

	private const int pistons = 6;

	private const int HARMONICS_COUNT = 3;

	private const int WAVE_FORM_COUNT = 4;

	[Export]
	private NodePath carEngine;

	[Export(PropertyHint.Range, "0, 1000")]
	private float minRpm = DEFAULT_MIN_RPM;

	[Export(PropertyHint.Range, "1000, 10000")]
	private float maxRpm = DEFAULT_MAX_RPM;

	[Export(PropertyHint.Range, "4, 12,")]
	private int cylinderCount = 6;

	[Export(PropertyHint.ExpRange, "30, 20000")]
	private float noiseLowPassCutoff = 10000f;

	[Export(PropertyHint.ExpRange, "30, 20000")]
	private float vibrationsLowPassCutoff = 10000f;

	[Export(PropertyHint.ExpRange, "30, 20000")]
	private float pistonRandomnessLowPassCutoff = 10000f;

	[Export(PropertyHint.Range, "0, 1.0")]
	private float intakeVolume = 0.5f;

	[Export(PropertyHint.Range, "0, 1.0")]
	private float vibrationsVolume = 0.5f;

	[Export(PropertyHint.Range, "0, 1.0")]
	private float exhaustVolume = 0.5f;

	[Export(PropertyHint.Range, "0, 1.0")]
	private float pistonTimingRandomness = 0.05f;

	[Export(PropertyHint.Range, "0, 5.0")]
	private float straightPipeLengthMeters = 3f;

	[Export(PropertyHint.Range, "-1.0, 1.0")]
	private float mufflersFwdReflToPipe = 0.5f;

	[Export(PropertyHint.Range, "-1.0, 1.0")]
	private float mufflersRevReflToAir = -0.5f;

	float phase = 0;
	float secondPhase = 0;

	private AudioStreamGeneratorPlayback playback;

	private float Sine(float phase) => (float)Mathf.Sin(phase);

	private float Triangle(float phase) => (float)(2.0 / Mathf.Pi * Mathf.Asin(Mathf.Sin(phase)));

	private float Saw(float phase) => (float)(2.0 * (phase / (Mathf.Tau) - Mathf.Floor(phase / (Mathf.Tau) + 0.5f)));

	private float Noise() => (float)(GD.Randf() * 2.0 - 1.0);

	private float[] harmonicGains = { 1.0f, 0.6f, 0.3f };

	private PinkNoise basePinkNoise = new PinkNoise();
	private PinkNoise pinkNoise = new PinkNoise();
	private PinkNoise envelopeNoise = new PinkNoise();

	private LowPassFilter noiseLowPass;

	private LowPassFilter vibrationsLowPass;

	private LowPassFilter pistonRnadomnessLowPass;

	// Alpha to Exhaust collector, beta to muffler elements
	private WaveGuide straightPipe;

	private WaveGuide[] mufflerElements;

	// Start with a single Cylinder
	private Cylinder[] cylinders;

	// Used to keep track of which cylinder group is firing when
	private float engineTime = 0f;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		noiseLowPass = new LowPassFilter(noiseLowPassCutoff, sampleRate);
		vibrationsLowPass = new LowPassFilter(vibrationsLowPassCutoff, sampleRate);
		pistonRnadomnessLowPass = new LowPassFilter(pistonRandomnessLowPassCutoff, sampleRate);
		straightPipe = new WaveGuide(toDelayTime(straightPipeLengthMeters), 6.173e-2f, 1.65e-3f, sampleRate);
		initCylinders();
		initMufflerElements();
		AudioStreamPlayer player = GetNode<AudioStreamPlayer>("Player");
		playback = (AudioStreamGeneratorPlayback)player.GetStreamPlayback();
		FillBuffer(0);
		player.Play();
	}

	private void initCylinders()
	{
		cylinders = new Cylinder[cylinderCount];

		for (int i = 0; i < cylinderCount; i++)
		{
			cylinders[i] = new Cylinder(sampleRate, 0.25f, i);
		}
	}

	private void initMufflerElements()
	{
		mufflerElements = new WaveGuide[4] {
			new WaveGuide(0.0125f, -0.5f, 0.5f, sampleRate),
			new WaveGuide(1.2919e-4f, -0.5f, 0.5f, sampleRate),
			new WaveGuide(0.0025f, -0.5f, 0.5f, sampleRate),
			new WaveGuide(0.002f, -0.5f, 0.5f, sampleRate)
		};
	}

	private float toDelayTime(float lengthMeters)
	{
		// 340.29 m/s speed of sound in thin Air
		return lengthMeters / 340.29f;
	}

	//  // Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(float delta)
	{
		noiseLowPass.setCutOff(noiseLowPassCutoff);
		vibrationsLowPass.setCutOff(vibrationsLowPassCutoff);
		pistonRnadomnessLowPass.setCutOff(pistonRandomnessLowPassCutoff);

		for (int i = 0; i < mufflerElements.Length; i++)
		{
			mufflerElements[i].fwdOutRefl = mufflersFwdReflToPipe;
			mufflerElements[i].revOutRefl = mufflersRevReflToAir;
		}

		FillBuffer(delta);
	}
	private float[] phases = new float[HARMONICS_COUNT * WAVE_FORM_COUNT];

	// waveForm -> 0 = Triangle, 1 = Sine, 2 = Noise modulated Sine, 3 = Sawtooth
	private float getWaveFormPhase(int harmonic, int waveForm)
	{
		return phases[(harmonic - 1) * WAVE_FORM_COUNT + waveForm];
	}

	private void updateWaveFormPhase(int harmonic, int waveForm, float phaseChange)
	{
		int waveFormIndex = (harmonic - 1) * WAVE_FORM_COUNT + waveForm;
		phases[waveFormIndex] += phaseChange;

		if (phases[waveFormIndex] > Mathf.Tau)
		{
			phases[waveFormIndex] -= Mathf.Tau;
		}
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

	private float postionMotion(float crankPos)
	{
		return Mathf.Cos(2 * Mathf.Tau * crankPos);
	}

	// t - Time(relative to full cycle) needed by the fuel to explode
	private float fuelIgnition(float crankPos, float t)
	{
		if (crankPos < 0 || crankPos > t)
		{
			return 0;
		}

		return Mathf.Sin(Mathf.Tau * (crankPos * t + .5f));
	}

	// Move to an "Engine" class
	float crankPos = 0;

	private void generateData(Vector2[] buffer, int framesAvailble)
	{
		float currentRpm = (float)GetNode(carEngine).Get("currentRpm");

		float throttle = (float)GetNode(carEngine).Get("throttle");

		float freq = currentRpm / 120f; // pistons fire once every two cycles and converting (per minute) to (per second)

		float crankPosInc = freq / sampleRate;

		for (int i = 0; i < framesAvailble; i++)
		{
			crankPos = (crankPos + crankPosInc) % 1.0f;

			float sample = 0;
			float intake = 0;
			float vibrations = 0;
			float exhaust = 0;

			WaveGuideOutput totalMuffletWGOut = new WaveGuideOutput();

			for (int j = 0; j < mufflerElements.Length; j++)
			{
				WaveGuideOutput elOut = mufflerElements[j].Pop();
				totalMuffletWGOut.fwdChamberOut += elOut.fwdChamberOut;
				totalMuffletWGOut.revChamberOut += elOut.revChamberOut;
			}

			float mufflerOut = totalMuffletWGOut.revChamberOut;

			WaveGuideOutput straightPipeWGoutput = straightPipe.Pop();
			float pipeToExtractors = straightPipeWGoutput.fwdChamberOut / cylinders.Length;

			// This ensures equal distribution of pistons firing over time, if the distribution is not equal we will get growling
			float pistonOffset = 1f / cylinders.Length;

			for (int j = 0; j < cylinders.Length; j++)
			{
				float crankOffset = (j + 0.5f) * pistonOffset;
				float pistonRandomness = .5f * pistonTimingRandomness * Mathf.Sin(Mathf.Tau * crankOffset)/ cylinders.Length;
				float cylinderPos = crankPos + crankOffset + pistonRandomness;
				CylinderOut cylinderOut = cylinders[j].Write(cylinderPos, throttle, noiseLowPass.filter(Noise()), pipeToExtractors);
				intake += cylinderOut.intakeOut;
				vibrations += cylinderOut.vibrationsOut;
				exhaust += cylinderOut.exhaust;
				// GD.Print($"Cylinder {j} Exhaust: {cylinderOut.exhaust}");
			}


			straightPipe.Push(totalMuffletWGOut.fwdChamberOut, exhaust);

			float pipeToMufflers = straightPipeWGoutput.revChamberOut / mufflerElements.Length;

			for (int j = 0; j < mufflerElements.Length; j++)
			{
				mufflerElements[j].Push(0, pipeToMufflers);
			}

			// GD.Print($"Muffler out: {mufflerOut:F7}");

			// sample += intake * intakeVolume + vibrationsLowPass.filter(vibrations) * vibrationsVolume + mufflerOut * 10000;
			sample += intake * intakeVolume + vibrationsLowPass.filter(vibrations) * vibrationsVolume + mufflerOut * exhaustVolume;


			buffer[i] = Vector2.One * sample;
		}
	}

	private void FillBuffer(float delta)
	{

		int framesAvailable = playback.GetFramesAvailable();

		if (framesAvailable == 0)
		{
			return;
		}

		Vector2[] buffer = new Vector2[framesAvailable];

		generateData(buffer, framesAvailable);

		if (framesAvailable > 0)
		{
			playback.PushBuffer(buffer);
		}
	}

	private float GetBaseFreq(float currentRpm)
	{
		// Use rpm, and cylinder firing order to calculate "explosions per second" to get he dominant frequency
		float rps = currentRpm / 60.0f;
		int firingPistonsPerRound = pistons / 2;
		return rps * firingPistonsPerRound;
	}
}
