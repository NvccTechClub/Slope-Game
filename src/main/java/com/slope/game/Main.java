package com.slope.game;

class Main {
    private static Game game;
    private static Settings settings;

    public static void main(String[] args) throws Exception {
        boolean runApp = true;

        // Settings are read before anything exists that depends on them: the window is
        // created at the stored size, and the game picks its bindings up from here.
        settings = new Settings();
        settings.load();

        game = new Game();

        if (args.length > 0) {
            String mode = args[0];

            switch(mode) {
                case "workflowtest":
                    runApp = false;
            }
        }

        Engine.init();
        Engine.getMain().start();

        if(runApp) {
            try {
                Engine.getMain().run();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        Engine.getMain().destroy();
    }

    public static Game getGame() {
        return game;
    }

    public static Settings getSettings() {
        if (settings == null) {
            // Only reachable if something asks before main() has run; defaults are fine.
            settings = new Settings();
        }

        return settings;
    }
}