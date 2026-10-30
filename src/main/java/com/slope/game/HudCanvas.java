package com.slope.game;

import java.nio.ByteBuffer;

import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL12;
import org.lwjgl.system.MemoryUtil;

/**
 * A software-drawn overlay. Everything the player reads — score, prompts, menus, the
 * debug readout — is drawn here as pixels and handed to the screen pass as one texture.
 *
 * <p>Drawing on the CPU rather than in GLSL is what makes arbitrary text and menu layout
 * possible at all: the old HUD could only ever show numbers, because each glyph had to
 * be an expression in the fragment shader. The cost is one texture upload per frame the
 * overlay actually changed, which is why every draw call marks the canvas dirty and
 * {@link #upload()} does nothing when it is not.
 *
 * <p>Coordinates are top-left origin, y increasing downwards, in virtual pixels. The
 * canvas is a fixed size regardless of window size, so layout never has to be rewritten
 * for a different resolution; the screen pass stretches it.
 */
public final class HudCanvas {
    /** Virtual resolution. Small enough that a full upload is cheap, large enough to read. */
    public static final int WIDTH = 640;
    public static final int HEIGHT = 360;

    private static final int STRIDE = WIDTH * 4;

    private final ByteBuffer pixels;
    private int texture = 0;
    private boolean dirty = true;

    public HudCanvas() {
        pixels = MemoryUtil.memCalloc(WIDTH * HEIGHT * 4);
    }

    /** Creates the GL texture. Needs a current context, so call during init. */
    public void init() {
        texture = GL11.glGenTextures();
        GL11.glBindTexture(GL11.GL_TEXTURE_2D, texture);

        // Nearest keeps the 5x7 glyphs crisp when the canvas is scaled up to the window.
        GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MIN_FILTER, GL11.GL_NEAREST);
        GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MAG_FILTER, GL11.GL_NEAREST);
        GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_WRAP_S, GL12.GL_CLAMP_TO_EDGE);
        GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_WRAP_T, GL12.GL_CLAMP_TO_EDGE);

        GL11.glTexImage2D(GL11.GL_TEXTURE_2D, 0, GL11.GL_RGBA, WIDTH, HEIGHT, 0,
                GL11.GL_RGBA, GL11.GL_UNSIGNED_BYTE, pixels);

        GL11.glBindTexture(GL11.GL_TEXTURE_2D, 0);
        dirty = false;
    }

    /** Pushes the canvas to the GPU if anything was drawn since the last upload. */
    public void upload() {
        if (!dirty || texture == 0) {
            return;
        }

        GL11.glBindTexture(GL11.GL_TEXTURE_2D, texture);
        GL11.glTexSubImage2D(GL11.GL_TEXTURE_2D, 0, 0, 0, WIDTH, HEIGHT,
                GL11.GL_RGBA, GL11.GL_UNSIGNED_BYTE, pixels);
        GL11.glBindTexture(GL11.GL_TEXTURE_2D, 0);

        dirty = false;
    }

    public int getTexture() {
        return texture;
    }

    /**
     * The raw RGBA pixels, for diagnostics that want to inspect or save what was drawn
     * without a GPU in the way. Not for drawing: use the draw methods, which keep the
     * dirty flag honest.
     */
    ByteBuffer pixels() {
        return pixels;
    }

    public void destroy() {
        if (texture != 0) {
            GL11.glDeleteTextures(texture);
            texture = 0;
        }

        MemoryUtil.memFree(pixels);
    }

    /** Wipes the canvas to fully transparent. */
    public void clear() {
        // Writing zeroed longs is noticeably quicker than four byte puts per pixel.
        for (int i = 0; i < pixels.capacity(); i += 8) {
            pixels.putLong(i, 0L);
        }

        dirty = true;
    }

    /**
     * Blends one pixel using its source alpha. Out-of-bounds writes are dropped, so
     * callers may lay out freely without clipping every rectangle themselves.
     */
    private void blend(int x, int y, float r, float g, float b, float a) {
        if (x < 0 || y < 0 || x >= WIDTH || y >= HEIGHT || a <= 0f) {
            return;
        }

        int i = (y * STRIDE) + (x * 4);

        if (a >= 1f) {
            pixels.put(i, (byte) toByte(r));
            pixels.put(i + 1, (byte) toByte(g));
            pixels.put(i + 2, (byte) toByte(b));
            pixels.put(i + 3, (byte) 255);
            return;
        }

        float dstR = (pixels.get(i) & 0xFF) / 255f;
        float dstG = (pixels.get(i + 1) & 0xFF) / 255f;
        float dstB = (pixels.get(i + 2) & 0xFF) / 255f;
        float dstA = (pixels.get(i + 3) & 0xFF) / 255f;

        float outA = a + (dstA * (1f - a));
        float outR = (r * a) + (dstR * dstA * (1f - a));
        float outG = (g * a) + (dstG * dstA * (1f - a));
        float outB = (b * a) + (dstB * dstA * (1f - a));

        if (outA > 1e-5f) {
            outR /= outA;
            outG /= outA;
            outB /= outA;
        }

        pixels.put(i, (byte) toByte(outR));
        pixels.put(i + 1, (byte) toByte(outG));
        pixels.put(i + 2, (byte) toByte(outB));
        pixels.put(i + 3, (byte) toByte(outA));
    }

    private static int toByte(float v) {
        int b = (int) ((v * 255f) + 0.5f);
        return b < 0 ? 0 : (b > 255 ? 255 : b);
    }

    public void fillRect(int x, int y, int w, int h, float r, float g, float b, float a) {
        if (w <= 0 || h <= 0) {
            return;
        }

        int x0 = Math.max(0, x);
        int y0 = Math.max(0, y);
        int x1 = Math.min(WIDTH, x + w);
        int y1 = Math.min(HEIGHT, y + h);

        for (int py = y0; py < y1; py++) {
            for (int px = x0; px < x1; px++) {
                blend(px, py, r, g, b, a);
            }
        }

        dirty = true;
    }

    /** A one-pixel outline, drawn just inside the given rectangle. */
    public void strokeRect(int x, int y, int w, int h, float r, float g, float b, float a) {
        fillRect(x, y, w, 1, r, g, b, a);
        fillRect(x, y + h - 1, w, 1, r, g, b, a);
        fillRect(x, y, 1, h, r, g, b, a);
        fillRect(x + w - 1, y, 1, h, r, g, b, a);
    }

    /** A filled panel with a brighter border, the backing for every menu. */
    public void panel(int x, int y, int w, int h, float[] fill, float[] border) {
        fillRect(x, y, w, h, fill[0], fill[1], fill[2], fill[3]);
        strokeRect(x, y, w, h, border[0], border[1], border[2], border[3]);
    }

    /**
     * Draws text with its left edge at x and its top at y.
     *
     * @param scale integer pixel multiplier; 1 gives 5x7 glyphs
     * @return the x coordinate just past the drawn text
     */
    public int text(int x, int y, String str, int scale, float r, float g, float b, float a) {
        if (str == null || str.isEmpty() || scale < 1) {
            return x;
        }

        int penX = x;

        for (int i = 0; i < str.length(); i++) {
            int[] rows = Font5x7.glyph(str.charAt(i));

            for (int row = 0; row < Font5x7.GLYPH_HEIGHT; row++) {
                int mask = rows[row];

                if (mask == 0) {
                    continue;
                }

                for (int col = 0; col < Font5x7.GLYPH_WIDTH; col++) {
                    // Bit 4 is the leftmost pixel of the row.
                    if ((mask & (1 << (Font5x7.GLYPH_WIDTH - 1 - col))) == 0) {
                        continue;
                    }

                    fillRect(penX + (col * scale), y + (row * scale), scale, scale, r, g, b, a);
                }
            }

            penX += Font5x7.ADVANCE * scale;
        }

        dirty = true;
        return penX - scale;
    }

    /** Text with a hard one-pixel drop shadow, so it stays legible over bright scenery. */
    public int shadowText(int x, int y, String str, int scale,
                          float r, float g, float b, float a) {
        text(x + scale, y + scale, str, scale, 0f, 0f, 0f, a * 0.7f);
        return text(x, y, str, scale, r, g, b, a);
    }

    public void textCentered(int centreX, int y, String str, int scale,
                             float r, float g, float b, float a) {
        int w = Font5x7.measure(str) * scale;
        shadowText(centreX - (w / 2), y, str, scale, r, g, b, a);
    }

    public void textRight(int rightX, int y, String str, int scale,
                          float r, float g, float b, float a) {
        int w = Font5x7.measure(str) * scale;
        shadowText(rightX - w, y, str, scale, r, g, b, a);
    }

    public static int textWidth(String str, int scale) {
        return Font5x7.measure(str) * scale;
    }

    /**
     * A left-to-right meter. Used for speed, the time trial clock and the volume rows,
     * all of which read better as a length than as a number.
     */
    public void meter(int x, int y, int w, int h, float fraction,
                      float[] fill, float[] frame) {
        fraction = fraction < 0f ? 0f : (fraction > 1f ? 1f : fraction);

        fillRect(x, y, w, h, 0f, 0f, 0f, 0.45f);
        int filled = (int) ((w - 2) * fraction);
        fillRect(x + 1, y + 1, filled, h - 2, fill[0], fill[1], fill[2], fill[3]);
        strokeRect(x, y, w, h, frame[0], frame[1], frame[2], frame[3]);
    }
}
