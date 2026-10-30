package com.slope.game;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;

/**
 * The record of what the player has managed, kept per mode and across sessions. Each
 * mode holds a short leaderboard rather than a single number, so a good run still shows
 * up even when it is not a personal best.
 */
public final class HighScores {
    private static final String FILE_NAME = "scores.properties";
    private static final int BOARD_SIZE = 5;

    private final Map<GameMode, List<Integer>> boards = new EnumMap<>(GameMode.class);

    /** The daily board is only meaningful for one date; this is the one it holds. */
    private String dailyStamp = "";

    public HighScores() {
        for (GameMode mode : GameMode.values()) {
            boards.put(mode, new ArrayList<>());
        }
    }

    private static Path file() {
        return Settings.directory().resolve(FILE_NAME);
    }

    public void load(String todayStamp) {
        Path path = file();
        Properties props = new Properties();

        if (Files.isReadable(path)) {
            try (InputStream in = Files.newInputStream(path)) {
                props.load(in);
            } catch (IOException e) {
                System.err.println("Could not read high scores: " + e.getMessage());
                return;
            }
        }

        dailyStamp = props.getProperty("daily.stamp", "");

        for (GameMode mode : GameMode.values()) {
            List<Integer> board = boards.get(mode);
            board.clear();

            String raw = props.getProperty("board." + mode.name(), "");

            for (String piece : raw.split(",")) {
                String trimmed = piece.trim();

                if (trimmed.isEmpty()) {
                    continue;
                }

                try {
                    int value = Integer.parseInt(trimmed);

                    if (value > 0) {
                        board.add(value);
                    }
                } catch (NumberFormatException ignored) {
                    // A corrupt entry just does not count; the rest of the board survives.
                }
            }

            Collections.sort(board, Collections.reverseOrder());

            while (board.size() > BOARD_SIZE) {
                board.remove(board.size() - 1);
            }
        }

        // A new day means the daily run is a fresh challenge, so last day's board goes.
        if (!dailyStamp.equals(todayStamp)) {
            boards.get(GameMode.DAILY).clear();
            dailyStamp = todayStamp;
        }
    }

    public void save() {
        Properties props = new Properties();
        props.setProperty("daily.stamp", dailyStamp);

        for (GameMode mode : GameMode.values()) {
            StringBuilder sb = new StringBuilder();

            for (int value : boards.get(mode)) {
                if (sb.length() > 0) {
                    sb.append(',');
                }
                sb.append(value);
            }

            props.setProperty("board." + mode.name(), sb.toString());
        }

        try {
            Files.createDirectories(Settings.directory());

            try (OutputStream out = Files.newOutputStream(file())) {
                props.store(out, "Slope Game high scores");
            }
        } catch (IOException e) {
            System.err.println("Could not save high scores: " + e.getMessage());
        }
    }

    /** Files a finished run. Returns true when it is the mode's new best. */
    public boolean submit(GameMode mode, int score) {
        if (score <= 0) {
            return false;
        }

        List<Integer> board = boards.get(mode);
        boolean isBest = board.isEmpty() || score > board.get(0);

        board.add(score);
        Collections.sort(board, Collections.reverseOrder());

        while (board.size() > BOARD_SIZE) {
            board.remove(board.size() - 1);
        }

        return isBest;
    }

    public int getBest(GameMode mode) {
        List<Integer> board = boards.get(mode);
        return board.isEmpty() ? 0 : board.get(0);
    }

    public List<Integer> getBoard(GameMode mode) {
        return Collections.unmodifiableList(boards.get(mode));
    }
}
