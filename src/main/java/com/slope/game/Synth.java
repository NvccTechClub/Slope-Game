package com.slope.game;

import java.util.Random;

/**
 * Builds the game's sounds as raw PCM at load time. The project ships no audio files, and
 * generating the waveforms keeps it that way: no binary assets, no licences, and every
 * sound is a few lines that can be retuned by changing a number.
 *
 * <p>All output is 16-bit signed mono at {@link #SAMPLE_RATE}. Anything meant to loop is
 * crossfaded end-to-start so the seam is inaudible.
 */
final class Synth {
    static final int SAMPLE_RATE = 44100;

    private Synth() {}

    private static int samples(float seconds) {
        return Math.max(1, (int) (seconds * SAMPLE_RATE));
    }

    private static short clip(double v) {
        double scaled = v * 32767.0;
        return (short) (scaled > 32767.0 ? 32767 : (scaled < -32768.0 ? -32768 : scaled));
    }

    /** Crossfades the tail of a buffer over its head so it can loop without a click. */
    private static void makeSeamless(double[] data, int fade) {
        fade = Math.min(fade, data.length / 4);

        for (int i = 0; i < fade; i++) {
            double t = (double) i / fade;
            int tail = data.length - fade + i;
            data[i] = (data[i] * t) + (data[tail] * (1.0 - t));
        }

        // The faded tail has been folded into the head, so drop it from the loop body.
        for (int i = data.length - fade; i < data.length; i++) {
            data[i] = data[(i - (data.length - fade))];
        }
    }

    private static short[] finish(double[] data) {
        short[] out = new short[data.length];

        for (int i = 0; i < data.length; i++) {
            out[i] = clip(data[i]);
        }

        return out;
    }

    /** Fast one-pole low pass. Cutoff is a 0..1 coefficient, lower is darker. */
    private static void lowPass(double[] data, double coeff) {
        double last = 0.0;

        for (int i = 0; i < data.length; i++) {
            last += coeff * (data[i] - last);
            data[i] = last;
        }
    }

    private static void highPass(double[] data, double coeff) {
        double last = 0.0;

        for (int i = 0; i < data.length; i++) {
            double in = data[i];
            last += coeff * (in - last);
            data[i] = in - last;
        }
    }

    /**
     * The ball on the road: a bed of filtered noise with a little low tone under it.
     * Pitch and gain are driven by speed at playback time, so one loop covers the range.
     */
    static short[] rollLoop() {
        double[] data = new double[samples(1.2f)];
        Random rng = new Random(0xB01L);

        for (int i = 0; i < data.length; i++) {
            data[i] = (rng.nextDouble() * 2.0) - 1.0;
        }

        lowPass(data, 0.045);
        highPass(data, 0.004);

        // Normalise, since filtered noise comes out much quieter than it went in.
        double peak = 1e-9;
        for (double v : data) {
            peak = Math.max(peak, Math.abs(v));
        }

        double phase = 0.0;
        for (int i = 0; i < data.length; i++) {
            data[i] = (data[i] / peak) * 0.72;

            // A quiet 55 Hz body so the loop has weight, not just hiss.
            phase += (2.0 * Math.PI * 55.0) / SAMPLE_RATE;
            data[i] += Math.sin(phase) * 0.18;
        }

        makeSeamless(data, samples(0.08f));
        return finish(data);
    }

    /** The crash: a hard noise burst with a fast decay and a thump underneath. */
    static short[] impact() {
        double[] data = new double[samples(0.85f)];
        Random rng = new Random(0xDEADL);

        for (int i = 0; i < data.length; i++) {
            double t = (double) i / data.length;
            double env = Math.exp(-7.0 * t);
            double noise = (rng.nextDouble() * 2.0) - 1.0;

            // A pitch-dropping sine gives the hit a body rather than just a hiss.
            double thumpFreq = 140.0 * Math.exp(-3.0 * t);
            double thump = Math.sin((2.0 * Math.PI * thumpFreq * i) / SAMPLE_RATE);

            data[i] = ((noise * 0.6) + (thump * 0.8)) * env;
        }

        lowPass(data, 0.35);
        return finish(data);
    }

    /** Coin pickup: two quick rising blips, the usual shorthand for "you got it". */
    static short[] coin() {
        double[] data = new double[samples(0.18f)];
        double first = 988.0;   // B5
        double second = 1319.0; // E6
        int split = data.length / 3;

        for (int i = 0; i < data.length; i++) {
            double freq = (i < split) ? first : second;
            double t = (double) i / data.length;
            double env = Math.min(1.0, (1.0 - t) * 3.0);

            // Square-ish, for an arcade edge rather than a pure tone.
            double phase = (2.0 * Math.PI * freq * i) / SAMPLE_RATE;
            double square = Math.sin(phase) >= 0.0 ? 1.0 : -1.0;
            data[i] = ((square * 0.35) + (Math.sin(phase) * 0.45)) * env * 0.55;
        }

        lowPass(data, 0.5);
        return finish(data);
    }

    /** Boost pad: an upward sweep that lands on a held note. */
    static short[] boost() {
        double[] data = new double[samples(0.45f)];
        double phase = 0.0;

        for (int i = 0; i < data.length; i++) {
            double t = (double) i / data.length;
            double freq = 220.0 + (660.0 * Math.min(1.0, t * 1.6));
            phase += (2.0 * Math.PI * freq) / SAMPLE_RATE;

            double env = Math.min(1.0, t * 12.0) * Math.exp(-2.2 * t);
            double saw = ((phase / (2.0 * Math.PI)) % 1.0) * 2.0 - 1.0;
            data[i] = ((Math.sin(phase) * 0.6) + (saw * 0.3)) * env * 0.6;
        }

        lowPass(data, 0.45);
        return finish(data);
    }

    /** Leaving the ground: a short upward chirp. */
    static short[] jump() {
        double[] data = new double[samples(0.16f)];
        double phase = 0.0;

        for (int i = 0; i < data.length; i++) {
            double t = (double) i / data.length;
            double freq = 420.0 + (520.0 * t);
            phase += (2.0 * Math.PI * freq) / SAMPLE_RATE;
            data[i] = Math.sin(phase) * Math.exp(-5.0 * t) * 0.5;
        }

        return finish(data);
    }

    /** Touching down again: a soft low knock. */
    static short[] land() {
        double[] data = new double[samples(0.22f)];
        Random rng = new Random(0x1A4DL);

        for (int i = 0; i < data.length; i++) {
            double t = (double) i / data.length;
            double env = Math.exp(-11.0 * t);
            double noise = (rng.nextDouble() * 2.0) - 1.0;
            double thump = Math.sin((2.0 * Math.PI * 95.0 * i) / SAMPLE_RATE);
            data[i] = ((thump * 0.8) + (noise * 0.25)) * env * 0.7;
        }

        lowPass(data, 0.2);
        return finish(data);
    }

    /** Moving the menu cursor: a clean short blip. */
    static short[] uiMove() {
        double[] data = new double[samples(0.07f)];

        for (int i = 0; i < data.length; i++) {
            double t = (double) i / data.length;
            double phase = (2.0 * Math.PI * 660.0 * i) / SAMPLE_RATE;
            double square = Math.sin(phase) >= 0.0 ? 1.0 : -1.0;
            data[i] = square * Math.exp(-14.0 * t) * 0.3;
        }

        lowPass(data, 0.4);
        return finish(data);
    }

    /** Choosing a menu entry: the same blip, a fifth up and a touch longer. */
    static short[] uiSelect() {
        double[] data = new double[samples(0.13f)];

        for (int i = 0; i < data.length; i++) {
            double t = (double) i / data.length;
            double freq = (t < 0.4) ? 880.0 : 1320.0;
            double phase = (2.0 * Math.PI * freq * i) / SAMPLE_RATE;
            double square = Math.sin(phase) >= 0.0 ? 1.0 : -1.0;
            data[i] = square * Math.exp(-7.0 * t) * 0.32;
        }

        lowPass(data, 0.45);
        return finish(data);
    }

    /**
     * The music bed: a four-bar loop over a minor pentatonic, with a bass line on the
     * root and an arpeggio above it. Generated at an exact bar length so it loops cleanly.
     */
    static short[] musicLoop() {
        final double bpm = 132.0;
        final double beat = 60.0 / bpm;
        final int bars = 4;
        final int beatsPerBar = 4;
        final int totalSteps = bars * beatsPerBar * 2; // eighth notes
        final double stepSeconds = beat / 2.0;

        double[] data = new double[samples((float) (stepSeconds * totalSteps))];
        final int stepLen = data.length / totalSteps;

        // A minor pentatonic, two octaves, as semitone offsets from A2.
        final int[] scale = { 0, 3, 5, 7, 10, 12, 15, 17, 19, 22, 24 };
        final int[] bassPattern = { 0, 0, 7, 0, 5, 5, 3, 3 };
        final int[] leadPattern = {
            5, 7, 8, 7, 5, 4, 5, 7,
            8, 9, 8, 7, 5, 7, 4, 2,
            5, 7, 8, 10, 9, 8, 7, 5,
            4, 5, 7, 5, 3, 2, 1, 0,
        };

        for (int step = 0; step < totalSteps; step++) {
            int start = step * stepLen;

            // Bass: one note per half bar, held.
            int bassNote = bassPattern[(step / 4) % bassPattern.length];
            double bassFreq = 110.0 * Math.pow(2.0, bassNote / 12.0);

            int leadNote = scale[leadPattern[step % leadPattern.length] % scale.length];
            double leadFreq = 440.0 * Math.pow(2.0, leadNote / 12.0);

            for (int i = 0; i < stepLen && (start + i) < data.length; i++) {
                double t = (double) i / stepLen;

                // Lead: plucky, so the loop has rhythm instead of a drone.
                double leadEnv = Math.exp(-4.5 * t) * 0.9;
                double leadPhase = (2.0 * Math.PI * leadFreq * i) / SAMPLE_RATE;
                double lead = (Math.sin(leadPhase) * 0.6)
                        + ((Math.sin(leadPhase) >= 0.0 ? 1.0 : -1.0) * 0.2);

                // Bass: near-flat envelope across the step so it reads as held.
                double bassEnv = 0.85 - (0.2 * t);
                double bassPhase = (2.0 * Math.PI * bassFreq * (start + i)) / SAMPLE_RATE;
                double bass = Math.sin(bassPhase);

                // A soft tick on the beat, standing in for a kit.
                double tick = 0.0;
                if (step % 2 == 0 && t < 0.08) {
                    tick = Math.sin((2.0 * Math.PI * 180.0 * i) / SAMPLE_RATE)
                            * (1.0 - (t / 0.08)) * 0.5;
                }

                data[start + i] = (lead * leadEnv * 0.20)
                        + (bass * bassEnv * 0.30)
                        + (tick * 0.25);
            }
        }

        lowPass(data, 0.55);
        makeSeamless(data, samples(0.02f));
        return finish(data);
    }
}
