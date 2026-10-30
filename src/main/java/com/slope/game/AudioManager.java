package com.slope.game;

import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

import org.lwjgl.BufferUtils;
import org.lwjgl.openal.AL;
import org.lwjgl.openal.AL10;
import org.lwjgl.openal.ALC;
import org.lwjgl.openal.ALC10;
import org.lwjgl.openal.ALCCapabilities;

/**
 * The game's sound. Holds the OpenAL device, the generated buffers, a small pool of
 * one-shot sources, and the two continuous voices (roll, music) whose gain and pitch
 * track what the ball is doing.
 *
 * <p>Audio is optional at runtime. If no device can be opened — a headless machine, a
 * container, a broken driver — the manager reports itself disabled and every call becomes
 * a no-op rather than taking the game down with it.
 */
public final class AudioManager {
    /** One-shot voices available at once. Beyond this the oldest is reused. */
    private static final int SOURCE_POOL = 12;

    public enum Sfx { IMPACT, COIN, BOOST, JUMP, LAND, UI_MOVE, UI_SELECT }

    private final Map<Sfx, Integer> buffers = new EnumMap<>(Sfx.class);
    private final List<Integer> sources = new ArrayList<>();
    private final List<Integer> allBuffers = new ArrayList<>();

    private long device = 0L;
    private long context = 0L;
    private boolean enabled = false;
    private int nextSource = 0;

    // Continuous voices.
    private int rollSource = 0;
    private int musicSource = 0;

    private float masterVolume = 1f;
    private float musicVolume = 0.5f;
    private float sfxVolume = 1f;

    private boolean musicPlaying = false;

    public void init(Settings settings) {
        masterVolume = settings.getMasterVolume();
        musicVolume = settings.getMusicVolume();
        sfxVolume = settings.getSfxVolume();

        try {
            device = ALC10.alcOpenDevice((java.nio.ByteBuffer) null);

            if (device == 0L) {
                System.err.println("Audio: no output device, running silent.");
                return;
            }

            ALCCapabilities caps = ALC.createCapabilities(device);
            context = ALC10.alcCreateContext(device, (IntBuffer) null);

            if (context == 0L) {
                System.err.println("Audio: could not create context, running silent.");
                ALC10.alcCloseDevice(device);
                device = 0L;
                return;
            }

            ALC10.alcMakeContextCurrent(context);
            AL.createCapabilities(caps);

            buffers.put(Sfx.IMPACT, buffer(Synth.impact()));
            buffers.put(Sfx.COIN, buffer(Synth.coin()));
            buffers.put(Sfx.BOOST, buffer(Synth.boost()));
            buffers.put(Sfx.JUMP, buffer(Synth.jump()));
            buffers.put(Sfx.LAND, buffer(Synth.land()));
            buffers.put(Sfx.UI_MOVE, buffer(Synth.uiMove()));
            buffers.put(Sfx.UI_SELECT, buffer(Synth.uiSelect()));

            for (int i = 0; i < SOURCE_POOL; i++) {
                sources.add(AL10.alGenSources());
            }

            rollSource = loopingSource(buffer(Synth.rollLoop()));
            musicSource = loopingSource(buffer(Synth.musicLoop()));

            enabled = true;
            applyVolumes();
        } catch (Throwable t) {
            // A driver that throws should cost the player sound, not the game.
            System.err.println("Audio unavailable, running silent: " + t.getMessage());
            enabled = false;
        }
    }

    private int buffer(short[] pcm) {
        ShortBuffer data = BufferUtils.createShortBuffer(pcm.length);
        data.put(pcm).flip();

        int id = AL10.alGenBuffers();
        AL10.alBufferData(id, AL10.AL_FORMAT_MONO16, data, Synth.SAMPLE_RATE);
        allBuffers.add(id);
        return id;
    }

    private int loopingSource(int bufferId) {
        int src = AL10.alGenSources();
        AL10.alSourcei(src, AL10.AL_BUFFER, bufferId);
        AL10.alSourcei(src, AL10.AL_LOOPING, AL10.AL_TRUE);
        AL10.alSourcef(src, AL10.AL_GAIN, 0f);
        return src;
    }

    public boolean isEnabled() {
        return enabled;
    }

    /** Fires a one-shot. Quiet sounds are skipped rather than taking a source. */
    public void play(Sfx sfx) {
        play(sfx, 1f, 1f);
    }

    public void play(Sfx sfx, float gain, float pitch) {
        if (!enabled) {
            return;
        }

        Integer bufferId = buffers.get(sfx);

        if (bufferId == null) {
            return;
        }

        float finalGain = gain * sfxVolume * masterVolume;

        if (finalGain <= 0.001f) {
            return;
        }

        // Round-robin the pool. A one-shot that is still playing gets cut off only once
        // every other voice is busy, which at twelve sources is rare.
        int src = sources.get(nextSource);
        nextSource = (nextSource + 1) % sources.size();

        AL10.alSourceStop(src);
        AL10.alSourcei(src, AL10.AL_BUFFER, bufferId);
        AL10.alSourcef(src, AL10.AL_GAIN, finalGain);
        AL10.alSourcef(src, AL10.AL_PITCH, pitch <= 0f ? 1f : pitch);
        AL10.alSourcei(src, AL10.AL_LOOPING, AL10.AL_FALSE);
        AL10.alSourcePlay(src);
    }

    /**
     * Updates the continuous voices.
     *
     * @param speedFraction current speed over top speed, 0..1
     * @param airborne true while the ball is off the ground, which mutes the roll
     * @param active false in menus and on the game-over screen, which fades both out
     */
    public void updateMotion(float speedFraction, boolean airborne, boolean active) {
        if (!enabled) {
            return;
        }

        float f = clamp01(speedFraction);

        // Roll: present as soon as the ball moves, louder and brighter with speed.
        float rollGain = active && !airborne ? (0.18f + (0.55f * f)) : 0f;
        float rollPitch = 0.75f + (0.9f * f);

        setVoice(rollSource, rollGain * sfxVolume * masterVolume, rollPitch);
    }

    private void setVoice(int src, float gain, float pitch) {
        AL10.alSourcef(src, AL10.AL_GAIN, gain);
        AL10.alSourcef(src, AL10.AL_PITCH, pitch);

        int state = AL10.alGetSourcei(src, AL10.AL_SOURCE_STATE);

        if (gain > 0.001f) {
            if (state != AL10.AL_PLAYING) {
                AL10.alSourcePlay(src);
            }
        } else if (state == AL10.AL_PLAYING) {
            AL10.alSourcePause(src);
        }
    }

    public void startMusic() {
        if (!enabled || musicPlaying) {
            return;
        }

        musicPlaying = true;
        AL10.alSourcef(musicSource, AL10.AL_GAIN, musicVolume * masterVolume);
        AL10.alSourcePlay(musicSource);
    }

    public void stopMusic() {
        if (!enabled || !musicPlaying) {
            return;
        }

        musicPlaying = false;
        AL10.alSourceStop(musicSource);
    }

    /** Dips the music under the death sting, then back. 1 is normal. */
    public void setMusicDuck(float factor) {
        if (!enabled || !musicPlaying) {
            return;
        }

        AL10.alSourcef(musicSource, AL10.AL_GAIN,
                musicVolume * masterVolume * clamp01(factor));
    }

    /** Pushes volume changes from the settings screen onto the live voices. */
    public void applyVolumes(Settings settings) {
        masterVolume = settings.getMasterVolume();
        musicVolume = settings.getMusicVolume();
        sfxVolume = settings.getSfxVolume();
        applyVolumes();
    }

    private void applyVolumes() {
        if (!enabled) {
            return;
        }

        if (musicPlaying) {
            AL10.alSourcef(musicSource, AL10.AL_GAIN, musicVolume * masterVolume);
        }
    }

    public void destroy() {
        if (!enabled && device == 0L) {
            return;
        }

        try {
            for (int src : sources) {
                AL10.alSourceStop(src);
                AL10.alDeleteSources(src);
            }

            for (int src : new int[] { rollSource, musicSource }) {
                if (src != 0) {
                    AL10.alSourceStop(src);
                    AL10.alDeleteSources(src);
                }
            }

            for (int buf : allBuffers) {
                AL10.alDeleteBuffers(buf);
            }

            if (context != 0L) {
                ALC10.alcMakeContextCurrent(0L);
                ALC10.alcDestroyContext(context);
            }

            if (device != 0L) {
                ALC10.alcCloseDevice(device);
            }
        } catch (Throwable t) {
            System.err.println("Audio shutdown: " + t.getMessage());
        }

        sources.clear();
        allBuffers.clear();
        enabled = false;
        device = 0L;
        context = 0L;
    }

    private static float clamp01(float v) {
        return v < 0f ? 0f : (v > 1f ? 1f : v);
    }
}
