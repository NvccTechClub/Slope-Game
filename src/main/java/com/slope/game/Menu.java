package com.slope.game;

import java.util.ArrayList;
import java.util.List;
import java.util.function.IntConsumer;
import java.util.function.Supplier;

/**
 * A list of things the player can choose. Deliberately small: a menu is a title, some
 * rows, and a cursor. Each row knows how to describe itself and what to do when it is
 * picked or nudged left and right, which keeps every screen's behaviour next to the
 * screen that defines it rather than in a switch somewhere else.
 */
public final class Menu {
    /** One row. A row with an {@code onAdjust} is a setting; without one it is a button. */
    public static final class Item {
        final String label;
        final Supplier<String> value;
        final Runnable onSelect;
        final IntConsumer onAdjust;
        final boolean enabled;

        Item(String label, Supplier<String> value, Runnable onSelect,
             IntConsumer onAdjust, boolean enabled) {
            this.label = label;
            this.value = value;
            this.onSelect = onSelect;
            this.onAdjust = onAdjust;
            this.enabled = enabled;
        }

        public String getLabel() {
            return label;
        }

        /** The right-hand text for this row, or null when it has none. */
        public String getValue() {
            return value == null ? null : value.get();
        }

        public boolean isEnabled() {
            return enabled;
        }

        public boolean isAdjustable() {
            return onAdjust != null;
        }
    }

    private final String title;
    private final String subtitle;
    private final List<Item> items = new ArrayList<>();
    private int cursor = 0;

    public Menu(String title, String subtitle) {
        this.title = title;
        this.subtitle = subtitle;
    }

    /** A plain button. */
    public Menu button(String label, Runnable onSelect) {
        items.add(new Item(label, null, onSelect, null, true));
        return this;
    }

    /** A row showing a value, chosen with left and right. */
    public Menu option(String label, Supplier<String> value, IntConsumer onAdjust) {
        items.add(new Item(label, value, () -> onAdjust.accept(1), onAdjust, true));
        return this;
    }

    /** A row showing a value that is picked rather than adjusted, such as a key binding. */
    public Menu picker(String label, Supplier<String> value, Runnable onSelect) {
        items.add(new Item(label, value, onSelect, null, true));
        return this;
    }

    /** A row that is only there to be read. */
    public Menu label(String label, Supplier<String> value) {
        items.add(new Item(label, value, null, null, false));
        return this;
    }

    public String getTitle() {
        return title;
    }

    public String getSubtitle() {
        return subtitle;
    }

    public List<Item> getItems() {
        return items;
    }

    public int getCursor() {
        return cursor;
    }

    public Item getSelected() {
        return (cursor >= 0 && cursor < items.size()) ? items.get(cursor) : null;
    }

    /** Moves the cursor, skipping rows that cannot be chosen, and wrapping at the ends. */
    public boolean move(int delta) {
        if (items.isEmpty() || delta == 0) {
            return false;
        }

        int start = cursor;

        for (int step = 0; step < items.size(); step++) {
            cursor = Math.floorMod(cursor + delta, items.size());

            if (items.get(cursor).enabled) {
                return cursor != start;
            }
        }

        cursor = start;
        return false;
    }

    /** Puts the cursor on the first row that can actually be chosen. */
    public void resetCursor() {
        cursor = 0;

        for (int i = 0; i < items.size(); i++) {
            if (items.get(i).enabled) {
                cursor = i;
                return;
            }
        }
    }

    /** Runs the selected row's action. Returns true if something happened. */
    public boolean select() {
        Item item = getSelected();

        if (item == null || !item.enabled || item.onSelect == null) {
            return false;
        }

        item.onSelect.run();
        return true;
    }

    /** Nudges the selected row. Returns true if it was adjustable. */
    public boolean adjust(int delta) {
        Item item = getSelected();

        if (item == null || !item.enabled || item.onAdjust == null) {
            return false;
        }

        item.onAdjust.accept(delta);
        return true;
    }
}
