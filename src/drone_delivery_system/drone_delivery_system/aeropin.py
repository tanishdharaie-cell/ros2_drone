"""
aeropin.py  —  Gazebo-local DIGIPIN codec

Re-implements the official DIGIPIN algorithm (India Post / IIT-H) with
a custom bounding box that covers YOUR Gazebo world (home.sdf) instead
of India.  Same 16-char alphabet, same hierarchical 4-way subdivision,
same encode/decode logic — only the bounding box changes.

  Official DIGIPIN  →  India  100 km x 100 km  →  cell ≈ 4 m
  AEROPIN           →  Gazebo 100 m x 100 m   →  cell ≈ 0.0015 m  (0.15 cm)

World origin from home.sdf <spherical_coordinates>:
  lat = 47.47895°,  lon = 19.057785°  (Budapest)
  X = East, Y = North  (ENU frame, same as Gazebo default)

Format:  XXX-XXX-XX  (8 chars + 2 hyphens, e.g. K22-772-7T)

Quick reference — home.sdf objects:
  K22-222-22  →  Origin / drone spawn    ( 0.00,  0.00) m
  K2J-F64-3M  →  Person Standing         ( 4.40,  2.40) m
  K2K-97F-PM  →  Dumpster                ( 3.71,  4.45) m
  J54-95C-6C  →  Fire Hydrant            ( 0.45, -1.66) m
  J57-K47-PC  →  Cardboard Box stack     ( 2.39, -3.68) m
  8CT-PP5-CM  →  Table                   (-6.33,  5.25) m
  K22-772-7T  →  Test point (0.50, 0.50)
  K22-222-7K  →  Test point (0.01, 0.01)
"""

__all__ = ['encode', 'decode', 'validate',
           'WORLD_XMIN', 'WORLD_XMAX', 'WORLD_YMIN', 'WORLD_YMAX',
           'CELL_SIZE_M', 'CHARS']

# ── Constants ────────────────────────────────────────────────────────
CHARS       = '23456789CJKLMPFT'   # 16 symbols — identical to official DIGIPIN
LEVELS      = 8                    # subdivision steps → 8-char code
WORLD_XMIN  = -50.0                # West  boundary (metres)
WORLD_XMAX  =  50.0                # East  boundary (metres)
WORLD_YMIN  = -50.0                # South boundary (metres)
WORLD_YMAX  =  50.0                # North boundary (metres)
CELL_SIZE_M = (WORLD_XMAX - WORLD_XMIN) / (4 ** LEVELS)   # ≈ 0.00153 m


def encode(x: float, y: float) -> str:
    """
    Encode Gazebo local (x, y) metres → 8-char AEROPIN string.

    Raises ValueError if coordinates are outside world bounds.
    """
    if not (WORLD_XMIN <= x <= WORLD_XMAX):
        raise ValueError(
            f"x={x:.4f} m is outside world bounds "
            f"[{WORLD_XMIN}, {WORLD_XMAX}]")
    if not (WORLD_YMIN <= y <= WORLD_YMAX):
        raise ValueError(
            f"y={y:.4f} m is outside world bounds "
            f"[{WORLD_YMIN}, {WORLD_YMAX}]")

    xmin, xmax = WORLD_XMIN, WORLD_XMAX
    ymin, ymax = WORLD_YMIN, WORLD_YMAX
    code = []

    # ==========================================================
    # TODO 1
    #
    # Implement the hierarchical 4-way subdivision that turns
    # (x, y) into an 8-character AEROPIN code.
    #
    # Requirements:
    # - Repeat LEVELS times. On each iteration:
    #     • Split the current [xmin, xmax] x [ymin, ymax] box
    #       into a 4x4 grid (xstep, ystep).
    #     • Find which column/row (x, y) falls into — clamp to
    #       index 3 so a point exactly on the upper boundary
    #       doesn't fall outside the grid.
    #     • Narrow xmin/xmax and ymin/ymax to that sub-cell, so
    #       the next iteration subdivides further.
    #     • Append the character for that (col, row) to `code`,
    #       using CHARS[col * 4 + row].
    # - This is THE core operation of the whole AeroPin system —
    #   get this right and decode() becomes the mirror image of it.
    #
    # Walkthrough, one iteration at a time:
    #   1. xstep = (xmax - xmin) / 4.0          ystep similarly
    #      → width of ONE column/row in the current box.
    #   2. col = int((x - xmin) / xstep)
    #      → "how many step-widths past the left edge is x",
    #        truncated to an int 0..3 (number of cell-widths,
    #        i.e. integer division of the offset by the step).
    #      → clamp with min(col, 3): if x sits exactly on xmax,
    #        the raw division gives 4 (out of range) — clamping
    #        forces it into the last valid cell instead of
    #        crashing or producing a bad index later.
    #   3. row = min(int((y - ymin) / ystep), 3)   (same idea, Y axis)
    #   4. Narrow the box to that one sub-cell so next loop
    #      iteration zooms in further:
    #        xmin += col * xstep;  xmax = xmin + xstep
    #        ymin += row * ystep;  ymax = ymin + ystep
    #   5. code.append(CHARS[col * 4 + row])
    #      → there are 16 cells (4 cols x 4 rows) and 16 chars
    #        in CHARS, so col*4 + row flattens the 2D grid
    #        position into a single 0..15 index — same trick as
    #        flattening a 2D array into 1D.
    #
    # After LEVELS=8 loops you'll have 8 characters in `code`;
    # they get joined and hyphenated below.
    #
    # Hint:
    # Use:
    #   • LEVELS, CHARS
    #   • xmin, xmax, ymin, ymax (already initialized above)
    #   • code (list to append characters to)
    # ==========================================================

    # YOUR CODE HERE
    for _ in range(LEVELS):
        xstep = (xmax - xmin) / 4.0
        ystep = (ymax - ymin) / 4.0

        col = min(int((x - xmin) / xstep), 3)
        row = min(int((y - ymin) / ystep), 3)

        xmin = xmin + col * xstep
        xmax = xmin + xstep
        ymin = ymin + row * ystep
        ymax = ymin + ystep

        code.append(CHARS[col * 4 + row])

    s = ''.join(code)
    return f"{s[0:3]}-{s[3:6]}-{s[6:8]}"


def decode(code: str) -> tuple:
    """
    Decode an 8-char AEROPIN string → (x, y) centre of cell, in metres.

    Hyphens are ignored; input is case-insensitive.
    Raises ValueError on bad length or invalid characters.
    """
    clean = code.replace('-', '').upper()

    if len(clean) != LEVELS:
        raise ValueError(
            f"AEROPIN must be {LEVELS} characters "
            f"(got {len(clean)} from '{code}')")
    for ch in clean:
        if ch not in CHARS:
            raise ValueError(
                f"Invalid character '{ch}' in '{code}'. "
                f"Allowed: {CHARS}")

    xmin, xmax = WORLD_XMIN, WORLD_XMAX
    ymin, ymax = WORLD_YMIN, WORLD_YMAX

    # ==========================================================
    # TODO 2
    #
    # Implement the inverse of TODO 1: walk through each
    # character of the cleaned code and narrow the bounding box
    # down to the final cell it represents.
    #
    # Requirements:
    # - For each character in `clean`:
    #     • Find its index in CHARS, then recover (col, row)
    #       from that index — the inverse of
    #       CHARS[col * 4 + row] used in encode().
    #     • Split the current box into a 4x4 grid (xstep, ystep),
    #       same as in encode().
    #     • Narrow xmin/xmax and ymin/ymax to the sub-cell given
    #       by (col, row).
    # - After the loop, xmin/xmax/ymin/ymax describe the final
    #   ~1.5mm cell — the function then returns its centre point.
    #
    # Walkthrough, one character at a time:
    #   1. idx = CHARS.index(ch)
    #      → recovers the SAME number encode() used:
    #        col*4 + row.
    #   2. col = idx // 4
    #      row = idx % 4
    #      → unflattening a 1D index back into 2D: integer
    #        division tells you which group-of-4 you're in
    #        (the column), the remainder tells you your
    #        position within that group (the row). This is the
    #        precise inverse of "col*4 + row" from encode().
    #   3. xstep = (xmax - xmin) / 4.0   ystep similarly
    #      → identical to encode(); must use the SAME box-
    #        splitting math or the two functions won't agree.
    #   4. Narrow the box using the recovered col/row, exactly
    #      like encode() did with its freshly computed col/row:
    #        xmin += col * xstep;  xmax = xmin + xstep
    #        ymin += row * ystep;  ymax = ymin + ystep
    #
    # Do this once per character (8 total) and the box shrinks
    # down to the same tiny cell the original point landed in.
    # That's why decode(encode(x,y)) won't give back the exact
    # x,y — only the centre of that final ~1.5mm cell (see the
    # `err` value printed by the CLI block below).
    #
    # Hint:
    # Use:
    #   • clean, CHARS
    #   • xmin, xmax, ymin, ymax (already initialized above)
    #   • Integer division/modulo to split an index back into
    #     (col, row) — the reverse of col * 4 + row.
    # ==========================================================

    # YOUR CODE HERE
    for ch in clean:
        idx = CHARS.index(ch)
        col = idx // 4
        row = idx % 4

        xstep = (xmax - xmin) / 4.0
        ystep = (ymax - ymin) / 4.0

        xmin = xmin + col * xstep
        xmax = xmin + xstep
        ymin = ymin + row * ystep
        ymax = ymin + ystep

    return (xmin + xmax) / 2.0, (ymin + ymax) / 2.0


def validate(code: str) -> tuple:
    """Return (True, '') on success or (False, error_message) on failure."""
    try:
        decode(code)
        return True, ''
    except ValueError as e:
        return False, str(e)


# ── CLI ──────────────────────────────────────────────────────────────
if __name__ == '__main__':
    import sys, math

    if len(sys.argv) == 3:
        try:
            x, y  = float(sys.argv[1]), float(sys.argv[2])
            code  = encode(x, y)
            dx, dy = decode(code)
            err   = math.sqrt((dx - x) ** 2 + (dy - y) ** 2) * 100
            print(f"AEROPIN  : {code}")
            print(f"Input    : ({x:.4f}, {y:.4f}) m")
            print(f"Snaps to : ({dx:.4f}, {dy:.4f}) m  (error {err:.3f} cm)")
        except ValueError as e:
            print(f"Error: {e}", file=sys.stderr); sys.exit(1)

    elif len(sys.argv) == 2:
        try:
            x, y = decode(sys.argv[1])
            print(f"AEROPIN  : {sys.argv[1].upper()}")
            print(f"X        : {x:.6f} m")
            print(f"Y        : {y:.6f} m")
        except ValueError as e:
            print(f"Error: {e}", file=sys.stderr); sys.exit(1)

    else:
        print(f"Usage:")
        print(f"  python aeropin.py <x> <y>     # encode")
        print(f"  python aeropin.py <AEROPIN>   # decode")
        print(f"Cell size : {CELL_SIZE_M*100:.4f} cm")
        print(f"Bounds    : X/Y ∈ [{WORLD_XMIN}, {WORLD_XMAX}] m")