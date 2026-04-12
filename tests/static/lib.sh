#!/usr/bin/env bash
# tests/static/lib.sh
# Shared helpers for the SlideSLAM ros2_dev static-check runner.
# Sourced by check_ros2_port.sh. POSIX-ish bash, no external deps beyond rg/grep/find/awk.

# ----- color codes (degrade if NO_COLOR is set or stdout isn't a tty) -----
if [ -n "${NO_COLOR:-}" ] || [ ! -t 1 ]; then
    C_RED=""
    C_GREEN=""
    C_YELLOW=""
    C_BLUE=""
    C_BOLD=""
    C_RESET=""
else
    C_RED=$'\033[31m'
    C_GREEN=$'\033[32m'
    C_YELLOW=$'\033[33m'
    C_BLUE=$'\033[34m'
    C_BOLD=$'\033[1m'
    C_RESET=$'\033[0m'
fi

# ----- counters (must be global; sourced into the runner script) -----
TOTAL=0
PASSED=0
FAILED=0
SKIPPED=0
FAILED_NAMES=()

VERBOSE="${VERBOSE:-0}"

# ----- output helpers -----
pass() {
    # $1 = check name
    TOTAL=$((TOTAL + 1))
    PASSED=$((PASSED + 1))
    printf "%s[PASS]%s %s\n" "${C_GREEN}" "${C_RESET}" "$1"
}

fail() {
    # $1 = check name, $2 (optional) = message, rest of args = sample hits
    TOTAL=$((TOTAL + 1))
    FAILED=$((FAILED + 1))
    FAILED_NAMES+=("$1")
    printf "%s[FAIL]%s %s\n" "${C_RED}" "${C_RESET}" "$1"
    if [ -n "${2:-}" ]; then
        printf "       %s%s%s\n" "${C_YELLOW}" "$2" "${C_RESET}"
    fi
    if [ "$VERBOSE" = "1" ] && [ "$#" -gt 2 ]; then
        shift 2
        for line in "$@"; do
            printf "         %s\n" "$line"
        done
    fi
}

skip() {
    # $1 = check name, $2 = reason
    TOTAL=$((TOTAL + 1))
    SKIPPED=$((SKIPPED + 1))
    printf "%s[SKIP]%s %s%s\n" "${C_YELLOW}" "${C_RESET}" "$1" "${2:+ ($2)}"
}

section() {
    printf "\n%s%s===== %s =====%s\n" "${C_BOLD}" "${C_BLUE}" "$1" "${C_RESET}"
}

# ----- search backend selection -----
# Picks ripgrep if available, otherwise falls back to a grep wrapper.
pick_search_tool() {
    if command -v rg >/dev/null 2>&1; then
        SEARCH_TOOL="rg"
    else
        SEARCH_TOOL="grep"
        printf "%s[WARN]%s ripgrep (rg) not found; falling back to grep -r (slower).\n" \
            "${C_YELLOW}" "${C_RESET}" >&2
    fi
}

# search_files <pattern> <root> [glob1 glob2 ...]
# Emits "file:line:matched_text" on stdout. Newline-separated.
# When SEARCH_TOOL=rg uses ripgrep with -n. When grep, walks files via find.
search_files() {
    local pattern="$1"
    local root="$2"
    shift 2
    if [ "$SEARCH_TOOL" = "rg" ]; then
        local args=(-n --no-heading --color=never)
        for g in "$@"; do
            args+=(-g "$g")
        done
        rg "${args[@]}" -e "$pattern" "$root" 2>/dev/null || true
    else
        # crude fallback
        local include_args=()
        for g in "$@"; do
            include_args+=(--include="$g")
        done
        grep -rEn "${include_args[@]}" "$pattern" "$root" 2>/dev/null || true
    fi
}

# Helper: split a "file:line:text" record into (file, line, text), correctly
# handling Windows-style paths like "C:\Users\...\file.cpp:42:code". The output
# is three lines via three globals: REC_FILE, REC_LINE, REC_TEXT (in awk).
# Implementation strategy: find the FIRST colon that is NOT part of a drive
# letter prefix (i.e. position > 2 OR character before is not a letter), then
# the SECOND colon after that is the line/text boundary.
#
# We use it via an awk function defined inline below.

# strip_cpp_line_comments
# Reads stdin "file:line:text" (rg or grep -n format), removes any lines whose
# code part is just a // comment or whose match is preceded by //. Handles
# Windows drive-letter paths.
strip_cpp_line_comments() {
    awk '
    function split_record(rec,    p1, p2, drive_off) {
        # Skip a leading "C:" Windows drive prefix when looking for the
        # file/line separator.
        drive_off = 0
        if (length(rec) >= 2 && substr(rec, 2, 1) == ":" && substr(rec, 1, 1) ~ /[A-Za-z]/) {
            drive_off = 2
        }
        p1 = index(substr(rec, drive_off + 1), ":")
        if (p1 == 0) { REC_FILE = rec; REC_LINE = ""; REC_TEXT = ""; return }
        p1 = p1 + drive_off
        REC_FILE = substr(rec, 1, p1 - 1)
        rest = substr(rec, p1 + 1)
        p2 = index(rest, ":")
        if (p2 == 0) { REC_LINE = rest; REC_TEXT = ""; return }
        REC_LINE = substr(rest, 1, p2 - 1)
        REC_TEXT = substr(rest, p2 + 1)
    }
    {
        split_record($0)
        text = REC_TEXT
        idx = index(text, "//")
        if (idx > 0) {
            code = substr(text, 1, idx - 1)
        } else {
            code = text
        }
        # also handle inline /* ... */ on a single line (rare)
        gsub(/\/\*[^*]*\*\//, "", code)
        sub(/^[ \t]+/, "", code)
        if (code == "") next
        printf "%s:%s:%s\n", REC_FILE, REC_LINE, code
    }
    '
}

# strip_python_line_comments
# Reads stdin "file:line:text", drops # line comments and lines that are pure
# comments. Handles Windows drive-letter paths.
strip_python_line_comments() {
    awk '
    function split_record(rec,    p1, p2, drive_off) {
        drive_off = 0
        if (length(rec) >= 2 && substr(rec, 2, 1) == ":" && substr(rec, 1, 1) ~ /[A-Za-z]/) {
            drive_off = 2
        }
        p1 = index(substr(rec, drive_off + 1), ":")
        if (p1 == 0) { REC_FILE = rec; REC_LINE = ""; REC_TEXT = ""; return }
        p1 = p1 + drive_off
        REC_FILE = substr(rec, 1, p1 - 1)
        rest = substr(rec, p1 + 1)
        p2 = index(rest, ":")
        if (p2 == 0) { REC_LINE = rest; REC_TEXT = ""; return }
        REC_LINE = substr(rest, 1, p2 - 1)
        REC_TEXT = substr(rest, p2 + 1)
    }
    {
        split_record($0)
        text = REC_TEXT
        idx = index(text, "#")
        if (idx > 0) {
            code = substr(text, 1, idx - 1)
        } else {
            code = text
        }
        sub(/^[ \t]+/, "", code)
        if (code == "") next
        printf "%s:%s:%s\n", REC_FILE, REC_LINE, code
    }
    '
}

# assert_no_match <check_name> <pattern> <root> <comment_lang> <glob...>
# comment_lang: "cpp" | "py" | "none"
# Runs the search, strips comments according to comment_lang, then re-applies
# the pattern via grep -E to keep only lines whose CODE part still matches.
assert_no_match() {
    local check_name="$1"
    local pattern="$2"
    local root="$3"
    local lang="$4"
    shift 4
    local raw stripped hits
    raw="$(search_files "$pattern" "$root" "$@")"
    if [ -z "$raw" ]; then
        pass "$check_name"
        return 0
    fi
    case "$lang" in
        cpp) stripped="$(printf "%s\n" "$raw" | strip_cpp_line_comments)" ;;
        py)  stripped="$(printf "%s\n" "$raw" | strip_python_line_comments)" ;;
        *)   stripped="$raw" ;;
    esac
    if [ -z "$stripped" ]; then
        pass "$check_name"
        return 0
    fi
    # Re-filter: keep only lines whose code still matches the pattern.
    hits="$(printf "%s\n" "$stripped" | grep -E "$pattern" || true)"
    if [ -z "$hits" ]; then
        pass "$check_name"
        return 0
    fi
    local count
    count=$(printf "%s\n" "$hits" | wc -l | tr -d ' ')
    local sample
    sample=$(printf "%s\n" "$hits" | head -5)
    if [ "$VERBOSE" = "1" ]; then
        fail "$check_name" "$count code-line hit(s) (showing up to 5):" $(printf "%s\n" "$sample")
    else
        fail "$check_name" "$count code-line hit(s) — rerun with --verbose to see them"
    fi
    return 1
}

# print_summary
print_summary() {
    printf "\n%s%s===== SUMMARY =====%s\n" "${C_BOLD}" "${C_BLUE}" "${C_RESET}"
    printf "%s%d checks, %s%d passed%s, %s%d failed%s, %s%d skipped%s\n" \
        "${C_BOLD}" "$TOTAL" \
        "${C_GREEN}" "$PASSED" "${C_RESET}" \
        "${C_RED}" "$FAILED" "${C_RESET}" \
        "${C_YELLOW}" "$SKIPPED" "${C_RESET}"
    if [ "$FAILED" -gt 0 ]; then
        printf "%sFailed checks:%s\n" "${C_RED}" "${C_RESET}"
        for n in "${FAILED_NAMES[@]}"; do
            printf "  - %s\n" "$n"
        done
    fi
}
