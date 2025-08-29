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

	[Export(PropertyHint.Range, "0.0, 1.0")]
	private float triangleAmp = .75f;

	[Export(PropertyHint.Range, "0.0, 1.0")]
	private float sawAmp = .5f;

	[Export(PropertyHint.Range, "0.0, 1.0")]
	private float sineAmp = 1.0f;

	[Export(PropertyHint.Range, "0.0, 1.0")]
	private float sineNoiseAmp = 1.0f;

	[Export(PropertyHint.Range, "0.0, 1.0")]
	private float sineNoisePhaseShift = .3f;

	// [Export(PropertyHint.Range, "0.0, .125")]
	// private float noiseSineModulation = .01f;

	[Export(PropertyHint.Range, "0.0, .5")]
	private float sinePhaseShift = .01f;

	[Export(PropertyHint.Range, "0.0, 1.0")]
	private float finalNoiseContribution = .3f;

	// [Export(PropertyHint.Range, "0.0, 1.0")]
	// private float baseNoiseAmp = .3f;

	[Export(PropertyHint.Range, "0.0, 1.0")]
	private float baseMinNoise = .1f;

	[Export(PropertyHint.Range, "0.0, 1.0")]
	private float baseMaxNoise = .6f;

	[Export(PropertyHint.Range, "0.0, 1.0")]
	private float triangleFreqMod = .1f;

	[Export(PropertyHint.Range, "0.0, 1.0")]
	private float sineFreqMod = .1f;
	float phase = 0;

	private AudioStreamGeneratorPlayback playback;

	private float Sine(float phase) => (float)Mathf.Sin(phase);

	private float Triangle(float phase) => (float)(2.0 / Mathf.Pi * Mathf.Asin(Mathf.Sin(phase)));

	private float Saw(float phase) => (float)(2.0 * (phase / (Mathf.Tau) - Mathf.Floor(phase / (Mathf.Tau) + 0.5f)));

	private float Noise() => (float)(GD.Randf() * 2.0 - 1.0);

	private float[] harmonicGains = { 1.0f, 0.6f, 0.3f };

	private PinkNoise basePinkNoise = new PinkNoise();
	private PinkNoise pinkNoise = new PinkNoise();
	private PinkNoise envelopeNoise = new PinkNoise();

	// The size is calcualted from (1000/60 * 3) (50Hz) and then 2/50 * 44100 =~ 1764 round up to closest multiple of 2
	private RingBuffer generationBuffer = new RingBuffer(2048);

	// Used to keep track of which cylinder group is firing when
	private float engineTime = 0f;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		AudioStreamPlayer player = GetNode<AudioStreamPlayer>("Player");
		playback = (AudioStreamGeneratorPlayback)player.GetStreamPlayback();
		FillBuffer(0);
		player.Play();
	}

	//  // Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(float delta)
	{
		engineTime += delta;
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

	private void generateData(Vector2[] buffer, int framesAvailble)
	{

		float currentRpm = (float)GetNode(carEngine).Get("currentRpm");
		float baseFreq = GetBaseFreq(currentRpm);
		float normalizedRpm = (currentRpm - minRpm) / (float)(maxRpm - minRpm);

		float maxSample = 0;

		// int framesForFullCycle = (int)Math.Ceiling((2 / baseFreq) * sampleRate);

		float firingFrequency = currentRpm / 120f;

		int framesToFill = Math.Min(generationBuffer.availableSpace(), framesAvailble);

		float previousEngineTime = engineTime;

		for (int i = 0; i < framesToFill; i++)
		{
			engineTime += 1f / sampleRate;
			float baseNoise = basePinkNoise.NextSample() * (baseMinNoise + (baseMaxNoise - baseMinNoise) * normalizedRpm);
			float sample = 0.0f;

			for (int harmonic = 1; harmonic < HARMONICS_COUNT; harmonic++)
			{
				float currSample = 0;
				// float harmonicPhase = phases[harmonic - 1];
				float firePhase = (float)(engineTime * firingFrequency + harmonic / (float)HARMONICS_COUNT) % 1.0f;
				float envelope = 0.5f * (1 - Mathf.Cos(Mathf.Tau * firePhase)); // fast thump
				envelope = envelope * .9f + .1f;
				currSample += triangleAmp * envelope * Triangle(getWaveFormPhase(harmonic, 0));
				// currSample += triangleAmp * Triangle(getWaveFormPhase(harmonic, 0));
				currSample += sineAmp * Sine(getWaveFormPhase(harmonic, 1) + Mathf.Tau * sinePhaseShift);
				float noise = pinkNoise.NextSample();
				// float noise = Noise();
				currSample += (sineNoiseAmp * noise + (1 - sineNoiseAmp)) * Sine(getWaveFormPhase(harmonic, 2) + Mathf.Tau * noise * sineNoisePhaseShift *(.1f + .9f * normalizedRpm));
				currSample += sawAmp * Saw(getWaveFormPhase(harmonic, 3));
				// currSample *= harmonicGains[harmonic - 1];
				// currSample *= envelope;
				// float envelope = Mathf.Pow(1f - firePhase, 2f);
				// float envelope = Mathf.Lerp(1f, 0f, firePhase);
				sample += currSample;

				updateWaveFormPhase(harmonic, 0, Mathf.Tau * (baseFreq * harmonic + triangleFreqMod) * (1f / sampleRate));
				updateWaveFormPhase(harmonic, 1, Mathf.Tau * (baseFreq * harmonic + sineFreqMod) * (1f / sampleRate));
				updateWaveFormPhase(harmonic, 2, Mathf.Tau * baseFreq * harmonic * (1f / sampleRate));
				updateWaveFormPhase(harmonic, 3, Mathf.Tau * baseFreq * harmonic * (1f / sampleRate));
				// updateWaveFormPhase(harmonic, 0, ((Mathf.Tau + Mathf.Tau / harmonic * triangleConstant * normalizedRpm) * baseFreq * harmonic) / sampleRate);
				// updateWaveFormPhase(harmonic, 1, ((Mathf.Tau + Mathf.Tau / harmonic * sineConstant * normalizedRpm / 2) * baseFreq * harmonic) / sampleRate);
				// updateWaveFormPhase(harmonic, 2, ((Mathf.Tau + Mathf.Tau / harmonic * noise * normalizedRpm / 3) * baseFreq * harmonic) / sampleRate);
				// updateWaveFormPhase(harmonic, 3, ((Mathf.Tau + Mathf.Tau / harmonic * sawConstant * 3 * (1 - normalizedRpm)) * baseFreq * harmonic) / sampleRate);

			}
			
			// GD.Print($"min Env: {minEnv}");

			// float firePhase = (float)(engineTime * baseFreq) % 1.0f;
			// float envelope = Mathf.Exp(-5f * (1 + envelopeNoise.NextSample() * 0.2f) * firePhase); // fast thump
			// sample *= envelope;
			sample = baseNoise * finalNoiseContribution + sample * (1 - finalNoiseContribution);

			// sample = baseNoise * sample;

			maxSample = Mathf.Max(Mathf.Abs(sample), maxSample);

			// sample = Mathf.Clamp(sample, -1, 1);

			buffer[i] = new Vector2(sample, sample);
			// phase = (phase + (baseFreq / sampleRate)) % 1.0f;
		}

		// Reset to previous since next dt will include this time
		engineTime = previousEngineTime;
	}

	private void FillBuffer(float delta)
	{
		// float phase = 0;

		int framesAvailable = playback.GetFramesAvailable();
		// int framesForFullCycle = (int)Math.Ceiling((2 / baseFreq) * sampleRate);

		// int framesToFill = Math.Min(framesAvailable, framesForFullCycle);

		// if (framesForFullCycle > framesAvailable && framesAvailable > 0)
		// {
		// 	GD.Print($"It's bigger! {framesForFullCycle} {framesAvailable} ");
		// }

		if (framesAvailable == 0)
		{
			return;
		}

		Vector2[] buffer = new Vector2[framesAvailable];

		// if (generationBuffer.availableData() >= framesAvailable)
		// {

		// 	GD.Print($"1 Before read: {generationBuffer.availableData()}");

		// 	for (int i = 0; i < framesAvailable; i++)
		// 	{
		// 		buffer[i] = generationBuffer.read();
		// 	}
		// 	GD.Print($"1 After read: {generationBuffer.availableData()}");
		// }
		// else
		// {
		// 	int currentData = generationBuffer.availableData();

		// 	GD.Print($"2 Before read: {generationBuffer.availableData()}");
		// 	for (int i = 0; i < currentData; i++)
		// 	{
		// 		buffer[i] = generationBuffer.read();
		// 	}

		// 	GD.Print($"2 After read: {generationBuffer.availableData()}");

		// 	framesAvailable -= currentData;

		// 	int generatedFrames = generationBuffer.availableData();

		// 	GD.Print($"Generated:  {generationBuffer.availableData()}, Requested: {framesAvailable}");

		// 	for (int i = currentData; i < currentData + Math.Min(generatedFrames, framesAvailable); i++)
		// 	{
		// 		buffer[i] = generationBuffer.read();
		// 	}
		// }

		generateData(buffer, framesAvailable);


		// if (framesAvailable > 0)
		// {
		// 	GD.Print($" Frame time: {(framesAvailable / (float)sampleRate):F3}s");
		// }

		if (framesAvailable > 0)
		{
			// GD.Print($"Sample: {maxSample:F6}");
			playback.PushBuffer(buffer);
		}

		// GD.Print($"Filled {framesAvailable} frames, which is {(framesAvailable / (float)sampleRate).ToString("F3")}s, Skips {playback.GetSkips()}");

		// while (framesAvailable > 0)
		// {
		// 	float sample = Mathf.Sin(phase) * 0.125f;
		// 	phase = (phase + (baseFreq / sampleRate)) % 1.0f;

		// 	playback.PushFrame(new Vector2(sample, sample));
		// 	framesAvailable--;
		// }
	}

	private float GetBaseFreq(float currentRpm)
	{
		// Use rpm, and cylinder firing order to calculate "explosions per second" to get he dominant frequency
		float rps = currentRpm / 60.0f;
		int firingPistonsPerRound = pistons / 2;
		return rps * firingPistonsPerRound;
	}
}
