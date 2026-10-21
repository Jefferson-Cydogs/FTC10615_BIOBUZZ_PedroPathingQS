package org.firstinspires.ftc.teamcode.cydogs.core;

public enum Alliance {
    RED("red"),
    BLUE("blue");

    private final String tagColorWord;

    Alliance(String tagColorWord) {
        this.tagColorWord = tagColorWord;
    }

    /** Lowercase word that appears in this alliance's AprilTag names. */
    public String tagColorWord() {
        return tagColorWord;
    }

    public Alliance opposite() {
        return this == RED ? BLUE : RED;
    }
}