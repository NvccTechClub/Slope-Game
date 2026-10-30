package com.slope.game;

import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWFramebufferSizeCallbackI;
import org.lwjgl.glfw.GLFWVidMode;
import org.lwjgl.glfw.GLFWWindowSizeCallbackI;
import org.lwjgl.opengl.GL;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL21;
import org.lwjgl.system.MemoryUtil;

public class Window {
    private int width;
    private int height;

    // Where and how big the window was before going fullscreen, so leaving fullscreen
    // puts it back rather than guessing.
    private final int[] restoreX = new int[1];
    private final int[] restoreY = new int[1];
    private int restoreWidth;
    private int restoreHeight;
    private boolean fullscreen;
    private int[] framebufferWidth;
    private int[] framebufferHeight;
    private String title;
    private boolean vsync;
    private float aspectRatio;

    private GLFWFramebufferSizeCallbackI frambufferCallback;

    private long window;

    public Window(int width, int height, String title) {

        // Make sure window manager is initialized before creating the window.
        if(!WindowManager.isInitialized()) {
            throw new IllegalStateException("WindowManager must be initialized before creating a window.");
        }

        this.width = width;
        this.height = height;
        this.title = title;
        this.vsync = true;
        this.fullscreen = false;

        this.framebufferWidth = new int[1];
        this.framebufferHeight = new int[1];

        if(this.width <= 0 || this.height <= 0) {
            this.width = 800;
            this.height = 600;
        }

        this.aspectRatio = (float)width / (float)height;
    }

    public void create() {
        // Create the actual window component.
        window = GLFW.glfwCreateWindow(width, height, title, MemoryUtil.NULL, MemoryUtil.NULL);

        // Signal when window was resized.

        GLFWFramebufferSizeCallbackI bufferSizeCall = (window, width, height) -> {
            this.framebufferWidth[0] = width;
            this.framebufferHeight[0] = height;
            this.aspectRatio = (float)width / (float)height;

            if(this.frambufferCallback != null) {
                // Forward the real framebuffer size; the renderer sizes its targets from this.
                this.frambufferCallback.invoke(this.window, width, height);
            }
        };

        GLFWWindowSizeCallbackI windowSizeCall = (window, width, height) -> {
            this.width = width;
            this.height = height;
        };

        GLFW.glfwGetFramebufferSize(window, framebufferWidth, framebufferHeight);
        GLFW.glfwSetFramebufferSizeCallback(window, bufferSizeCall);
        GLFW.glfwSetWindowSizeCallback(window, windowSizeCall);
        bufferSizeCall.invoke(window, framebufferWidth[0], framebufferHeight[0]);

        // Since we aren't planning on starting the program with a fullscreened window, let's just center it.
        // Our video mode will be our primary monitor.
        GLFWVidMode videoMode = GLFW.glfwGetVideoMode(GLFW.glfwGetPrimaryMonitor());
        GLFW.glfwSetWindowPos(window, (videoMode.width() - width) >> 1, (videoMode.height() - height) >> 1);

        // Set our current context to be the window.
        GLFW.glfwMakeContextCurrent(window);

        GLFW.glfwSwapInterval(vsync ? 1 : 0);

        this.restoreWidth = width;
        this.restoreHeight = height;

        // Display our window.
        GLFW.glfwShowWindow(window);
    }

    public void setFramebufferSizeCall(GLFWFramebufferSizeCallbackI frambufferCallback) {
        this.frambufferCallback = frambufferCallback;
    }

    public boolean shouldClose() {
        return GLFW.glfwWindowShouldClose(window);
    }

    public void update() {
        GLFW.glfwSwapBuffers(window);
    }

    public void destroy() {
        GLFW.glfwDestroyWindow(window);
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public int getFramebufferWidth() {
        return framebufferWidth[0];
    }

    public int getFramebufferHeight() {
        return framebufferHeight[0];
    }

    public String getTitle() {
        return title;
    }

    public boolean isVsync() {
        return vsync;
    }

    /**
     * Turns vsync on or off on the live context. Off uncaps the swap rate, which is what
     * you want when measuring; on is the default, because tearing is worse than latency
     * in a game played with two keys.
     */
    public void setVsync(boolean value) {
        this.vsync = value;
        GLFW.glfwMakeContextCurrent(window);
        GLFW.glfwSwapInterval(value ? 1 : 0);
    }

    public boolean isFullscreen() {
        return fullscreen;
    }

    /**
     * Moves the window between borderless-fullscreen on the primary monitor and its
     * previous windowed placement. The framebuffer callback fires from this, so the
     * render targets resize themselves.
     */
    public void setFullscreen(boolean value) {
        if (value == fullscreen) {
            return;
        }

        long monitor = GLFW.glfwGetPrimaryMonitor();
        GLFWVidMode mode = GLFW.glfwGetVideoMode(monitor);

        if (mode == null) {
            return; // no monitor to go fullscreen onto; stay as we are
        }

        if (value) {
            // Remember the windowed placement first, or there is nothing to come back to.
            GLFW.glfwGetWindowPos(window, restoreX, restoreY);
            restoreWidth = width;
            restoreHeight = height;

            GLFW.glfwSetWindowMonitor(window, monitor, 0, 0,
                    mode.width(), mode.height(), mode.refreshRate());
        } else {
            int w = restoreWidth > 0 ? restoreWidth : 1280;
            int h = restoreHeight > 0 ? restoreHeight : 720;

            GLFW.glfwSetWindowMonitor(window, MemoryUtil.NULL,
                    restoreX[0], restoreY[0], w, h, GLFW.GLFW_DONT_CARE);
        }

        fullscreen = value;

        // Changing the monitor resets the swap interval on some drivers, so reassert it.
        setVsync(vsync);
    }

    public void toggleFullscreen() {
        setFullscreen(!fullscreen);
    }

    public long getWindow() {
        return window;
    }

    public float getAspectRatio() { return aspectRatio; }
    
    public boolean isKeyPressed(int keyCode) {
        return GLFW.glfwGetKey(window, keyCode) == GLFW.GLFW_PRESS;
    }

    public float[] getMousePosition() {
        double[] xPos = new double[1];
        double[] yPos = new double[1];
        GLFW.glfwGetCursorPos(window, xPos, yPos);
        return new float[] { (float) xPos[0], (float) yPos[0] };
    }

    public void setMousePosition(float x, float y) {
        GLFW.glfwSetCursorPos(window, x, y);
    }
}
