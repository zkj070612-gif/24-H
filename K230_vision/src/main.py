"""
K230 / CanMV steel-ball detector.

The program uses the cv_lite Hough-circle implementation included in the
Yahboom factory firmware.  It does not require a kmodel.

UART1 wiring used by the factory board support package:
    K230 IO9  (TX) -> controller RX
    K230 IO10 (RX) -> controller TX (optional for this program)
    K230 GND        -> controller GND

UART output:
    $BALLS,count,id,x,y,r,score,id,x,y,r,score...#\n

Each stable steel ball has an independent temporal track and persistent ID.
"""

import gc
import time

import cv_lite
import image
from media.display import Display
from media.sensor import CAM_CHN_ID_1

from libs.PipeLine import PipeLine
from ybUtils.YbUart import YbUart


# ---------------------------------------------------------------------------
# Camera and display
# ---------------------------------------------------------------------------

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
IMAGE_SHAPE = [FRAME_HEIGHT, FRAME_WIDTH]

# Only the lower-middle pickup area is inspected.  The values below were
# calibrated from the 2026-07-25 steel-ball screenshots and deliberately
# exclude the white cable at the far right and objects on the upper desk.
# Format: x, y, width, height.
DETECTION_ROI = (80, 250, 480, 220)

# The primary target is the accepted ball closest to the electromagnet pickup
# point.  Adjust these two coordinates after the camera is mounted on the car.
PICKUP_TARGET_X = FRAME_WIDTH // 2
PICKUP_TARGET_Y = 470

# Draw rejected Hough circles and their measurements in red.  Keep this enabled
# during calibration, then disable it for the competition display.
SHOW_REJECTED_CIRCLES = False
MAX_DEBUG_CIRCLES = 12


# ---------------------------------------------------------------------------
# Hough-circle parameters
# ---------------------------------------------------------------------------

# Real balls in the supplied frames produced radii around 11..21 pixels.  The
# white cable false positive was around 43..47 pixels, so it is excluded here.
MIN_RADIUS = 7
MAX_RADIUS = 26
MIN_CENTER_DISTANCE = 18

HOUGH_DP = 1
HOUGH_CANNY_THRESHOLD = 80

# Lower values find more circles but also create more false positives.
HOUGH_ACCUMULATOR_THRESHOLD = 18


# ---------------------------------------------------------------------------
# Steel appearance filter
# ---------------------------------------------------------------------------

# A steel ball is normally low-saturation (grey/silver) and contains a bright
# highlight plus a darker region.  Disable this temporarily while calibrating
# the circle-size parameters if lighting conditions are poor.
ENABLE_STEEL_APPEARANCE_FILTER = True

MIN_LUMA_CONTRAST = 24

# Absolute highlight brightness varies with camera exposure.  The supplied
# steel-ball frames contain valid balls below the old value of 95.
MIN_HIGHLIGHT_LUMA = 55

# Steel balls were s=7..8; colourful screen icons in negative frames reached
# about s=50.
MAX_AVERAGE_SATURATION = 35

# A real sphere should have a visible brightness transition around much of its
# circumference.  This rejects circular textures and isolated reflections that
# merely produce a Hough-circle response.
MIN_EDGE_DIFFERENCE = 14

# Some real balls measured e=75..81, but small highly reflective balls can make
# Hough overestimate the radius and then report a very low boundary percentage.
# Known cable/icon false positives are now excluded mainly by ROI and radius.
MIN_EDGE_SUPPORT_PERCENT = 10

# 16 unit directions, scaled by 1000, used to sample the circular boundary
# without requiring floating-point trigonometry on every frame.
CIRCLE_DIRECTIONS = (
    (1000, 0),
    (924, 383),
    (707, 707),
    (383, 924),
    (0, 1000),
    (-383, 924),
    (-707, 707),
    (-924, 383),
    (-1000, 0),
    (-924, -383),
    (-707, -707),
    (-383, -924),
    (0, -1000),
    (383, -924),
    (707, -707),
    (924, -383),
)


# ---------------------------------------------------------------------------
# Stability and communication
# ---------------------------------------------------------------------------

STABLE_FRAME_COUNT = 3

# At about 28 FPS this keeps a locked target for roughly 0.3 seconds when the
# Hough detector misses it briefly.
LOST_FRAME_COUNT = 8

# Each physical ball owns an independent track.  The tolerances are deliberately
# smaller than the usual spacing between balls in the supplied video.
TRACK_POSITION_TOLERANCE = 26
TRACK_RADIUS_TOLERANCE = 12
MAX_BALL_TRACKS = 16

UART_BAUDRATE = 115200
UART_REPORT_INTERVAL_MS = 250
GC_INTERVAL_FRAMES = 15


def clamp(value, lower, upper):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def point_in_roi(x, y):
    roi_x, roi_y, roi_w, roi_h = DETECTION_ROI
    return (
        x >= roi_x
        and y >= roi_y
        and x < roi_x + roi_w
        and y < roi_y + roi_h
    )


def circle_in_roi(x, y, radius):
    """Require the complete circle, not only its centre, to be in the ROI."""
    roi_x, roi_y, roi_w, roi_h = DETECTION_ROI
    margin = 2
    return (
        x - radius - margin >= roi_x
        and y - radius - margin >= roi_y
        and x + radius + margin < roi_x + roi_w
        and y + radius + margin < roi_y + roi_h
    )


def read_rgb(rgb_array, x, y):
    """Read one HWC RGB888 pixel from the ulab array."""
    x = clamp(int(x), 0, FRAME_WIDTH - 1)
    y = clamp(int(y), 0, FRAME_HEIGHT - 1)
    pixel = rgb_array[y, x]
    return int(pixel[0]), int(pixel[1]), int(pixel[2])


def read_luma(rgb_array, x, y):
    red, green, blue = read_rgb(rgb_array, x, y)
    return (77 * red + 150 * green + 29 * blue) >> 8


def boundary_edge_support(rgb_array, x, y, radius):
    """
    Measure how much of the proposed circumference has an inside/outside edge.

    Returns a percentage from 0 to 100.
    """
    inner_radius = max(2, int(radius * 0.72))
    outer_radius = max(inner_radius + 2, int(radius * 1.18))
    supported = 0

    for dir_x, dir_y in CIRCLE_DIRECTIONS:
        inner_x = x + (dir_x * inner_radius) // 1000
        inner_y = y + (dir_y * inner_radius) // 1000
        outer_x = x + (dir_x * outer_radius) // 1000
        outer_y = y + (dir_y * outer_radius) // 1000

        inside_luma = read_luma(rgb_array, inner_x, inner_y)
        outside_luma = read_luma(rgb_array, outer_x, outer_y)
        if abs(inside_luma - outside_luma) >= MIN_EDGE_DIFFERENCE:
            supported += 1

    return (supported * 100) // len(CIRCLE_DIRECTIONS)


def steel_appearance_score(rgb_array, x, y, radius):
    """
    Validate the grey/silver highlight pattern inside a detected circle.

    Returns:
        (accepted, score, contrast, average_saturation, edge_support, reason)
    """
    inner = max(2, int(radius * 0.55))
    diagonal = max(1, int(inner * 0.70))

    sample_offsets = (
        (0, 0),
        (inner, 0),
        (-inner, 0),
        (0, inner),
        (0, -inner),
        (diagonal, diagonal),
        (-diagonal, diagonal),
        (diagonal, -diagonal),
        (-diagonal, -diagonal),
    )

    min_luma = 255
    max_luma = 0
    saturation_sum = 0

    for dx, dy in sample_offsets:
        red, green, blue = read_rgb(rgb_array, x + dx, y + dy)
        luma = (77 * red + 150 * green + 29 * blue) >> 8
        saturation = max(red, green, blue) - min(red, green, blue)

        if luma < min_luma:
            min_luma = luma
        if luma > max_luma:
            max_luma = luma
        saturation_sum += saturation

    contrast = max_luma - min_luma
    average_saturation = saturation_sum // len(sample_offsets)
    edge_support = boundary_edge_support(rgb_array, x, y, radius)

    reason = "OK"
    if contrast < MIN_LUMA_CONTRAST:
        reason = "LOW_CONTRAST"
    elif max_luma < MIN_HIGHLIGHT_LUMA:
        reason = "NO_HIGHLIGHT"
    elif average_saturation > MAX_AVERAGE_SATURATION:
        reason = "TOO_COLORFUL"
    elif edge_support < MIN_EDGE_SUPPORT_PERCENT:
        reason = "WEAK_EDGE"

    accepted = reason == "OK"

    # This score is for ranking/debug display, not a neural-network confidence.
    score = 20
    score += min(30, contrast)
    score += min(20, max_luma // 10)
    score += min(20, edge_support // 4)
    score += max(0, 10 - average_saturation // 6)
    score = clamp(score, 0, 99)

    return (
        accepted,
        score,
        contrast,
        average_saturation,
        edge_support,
        reason,
    )


def find_steel_balls(rgb_array):
    """
    Return (accepted, rejected).

    Candidate format:
        (x, y, radius, score, contrast, saturation, edge_percent, reason)
    """
    raw_circles = cv_lite.rgb888_find_circles(
        IMAGE_SHAPE,
        rgb_array,
        HOUGH_DP,
        MIN_CENTER_DISTANCE,
        HOUGH_CANNY_THRESHOLD,
        HOUGH_ACCUMULATOR_THRESHOLD,
        MIN_RADIUS,
        MAX_RADIUS,
    )

    accepted_candidates = []
    rejected_candidates = []
    if raw_circles is None:
        return accepted_candidates, rejected_candidates

    for index in range(0, len(raw_circles), 3):
        x = int(raw_circles[index])
        y = int(raw_circles[index + 1])
        radius = int(raw_circles[index + 2])

        if not point_in_roi(x, y):
            continue

        if not circle_in_roi(x, y, radius):
            rejected_candidates.append(
                (x, y, radius, 0, 0, 0, 0, "OUTSIDE_ROI")
            )
            continue

        accepted, score, contrast, saturation, edge_support, reason = (
            steel_appearance_score(
                rgb_array, x, y, radius
            )
        )
        candidate = (
            x,
            y,
            radius,
            score,
            contrast,
            saturation,
            edge_support,
            reason,
        )

        if ENABLE_STEEL_APPEARANCE_FILTER and not accepted:
            rejected_candidates.append(candidate)
            continue

        accepted_candidates.append(candidate)

    return deduplicate_candidates(accepted_candidates), rejected_candidates


def deduplicate_candidates(candidates):
    """
    Remove multiple Hough circles produced around the same physical ball.

    Nearby real balls are retained because the duplicate-centre tolerance is
    intentionally much smaller than MIN_CENTER_DISTANCE.
    """
    unique = []

    for candidate in candidates:
        x, y, radius, score = candidate[0:4]
        duplicate_index = -1

        for index in range(len(unique)):
            other = unique[index]
            other_x, other_y, other_radius = other[0:3]
            centre_tolerance = max(5, min(radius, other_radius) // 3)
            dx = x - other_x
            dy = y - other_y
            if dx * dx + dy * dy <= centre_tolerance * centre_tolerance:
                duplicate_index = index
                break

        if duplicate_index < 0:
            unique.append(candidate)
        elif score > unique[duplicate_index][3]:
            unique[duplicate_index] = candidate

    return unique


class BallTrack:
    """Temporal state for one physical steel ball."""

    def __init__(self, track_id, candidate):
        self.track_id = track_id
        self.current = candidate
        self.hit_count = 1
        self.missed_count = 0

    def update(self, candidate):
        old_x, old_y, old_radius, old_score = self.current[0:4]
        old_contrast, old_sat, old_edge = self.current[4:7]
        x, y, radius, score = candidate[0:4]
        contrast, saturation, edge_support = candidate[4:7]

        # Exponential smoothing reduces Hough centre/radius jitter.
        self.current = (
            (old_x * 3 + x) // 4,
            (old_y * 3 + y) // 4,
            (old_radius * 3 + radius) // 4,
            (old_score * 3 + score) // 4,
            (old_contrast * 3 + contrast) // 4,
            (old_sat * 3 + saturation) // 4,
            (old_edge * 3 + edge_support) // 4,
            "OK",
        )
        self.hit_count += 1
        self.missed_count = 0

    def mark_missed(self):
        self.missed_count += 1

    def is_stable(self):
        return (
            self.hit_count >= STABLE_FRAME_COUNT
            and self.missed_count < LOST_FRAME_COUNT
        )

    def is_expired(self):
        return self.missed_count >= LOST_FRAME_COUNT


class MultiBallTracker:
    """
    Greedy nearest-neighbour tracker for several stationary/slow steel balls.

    Each ball has an independent hit/miss counter and stable ID.  A temporary
    miss therefore affects only that ball instead of resetting all detections.
    """

    def __init__(self):
        self.tracks = []
        self.next_track_id = 1

    def _match_cost(self, track, candidate):
        track_x, track_y, track_radius = track.current[0:3]
        x, y, radius = candidate[0:3]
        dx = x - track_x
        dy = y - track_y
        dr = radius - track_radius

        if (
            abs(dx) > TRACK_POSITION_TOLERANCE
            or abs(dy) > TRACK_POSITION_TOLERANCE
            or abs(dr) > TRACK_RADIUS_TOLERANCE
        ):
            return None

        return dx * dx + dy * dy + dr * dr * 4

    def update(self, candidates):
        unmatched_indices = []
        for index in range(len(candidates)):
            unmatched_indices.append(index)

        # Existing tracks get first choice, which preserves established IDs.
        for track in self.tracks:
            best_list_pos = -1
            best_cost = None

            for list_pos in range(len(unmatched_indices)):
                candidate_index = unmatched_indices[list_pos]
                cost = self._match_cost(track, candidates[candidate_index])
                if cost is not None and (best_cost is None or cost < best_cost):
                    best_cost = cost
                    best_list_pos = list_pos

            if best_list_pos < 0:
                track.mark_missed()
            else:
                candidate_index = unmatched_indices.pop(best_list_pos)
                track.update(candidates[candidate_index])

        # Remove tracks only after a grace period.
        alive_tracks = []
        for track in self.tracks:
            if not track.is_expired():
                alive_tracks.append(track)
        self.tracks = alive_tracks

        # Unmatched circles start independent tentative tracks.
        for candidate_index in unmatched_indices:
            if len(self.tracks) >= MAX_BALL_TRACKS:
                break
            self.tracks.append(
                BallTrack(self.next_track_id, candidates[candidate_index])
            )
            self.next_track_id += 1

        return self.get_stable_tracks()

    def get_stable_tracks(self):
        stable_tracks = []
        for track in self.tracks:
            if track.is_stable():
                stable_tracks.append(track)
        return stable_tracks

    def get_pending_tracks(self):
        pending_tracks = []
        for track in self.tracks:
            if not track.is_stable():
                pending_tracks.append(track)
        return pending_tracks


def choose_pickup_track(stable_tracks, locked_track_id):
    """Keep the current pickup ID; otherwise select the nearest stable ball."""
    if locked_track_id is not None:
        for track in stable_tracks:
            if track.track_id == locked_track_id:
                return track

    best_track = None
    best_cost = None

    for track in stable_tracks:
        x, y, _radius, score = track.current[0:4]
        dx = x - PICKUP_TARGET_X
        dy = y - PICKUP_TARGET_Y
        cost = dx * dx + dy * dy - score * 4
        if best_cost is None or cost < best_cost:
            best_cost = cost
            best_track = track

    return best_track


def send_ball_results(uart, stable_tracks):
    """
    Send every stable ball:
        $BALLS,count,id,x,y,r,score,id,x,y,r,score...#
    """
    message = "$BALLS,%d" % len(stable_tracks)
    for track in stable_tracks:
        x, y, radius, score = track.current[0:4]
        message += ",%d,%d,%d,%d,%d" % (
            track.track_id,
            x,
            y,
            radius,
            score,
        )
    message += "#\n"

    uart.send(message)
    print(message, end="")


def draw_overlay(
    osd,
    rejected_candidates,
    tracker,
    stable_tracks,
    primary_track,
):
    osd.clear()

    roi_x, roi_y, roi_w, roi_h = DETECTION_ROI
    osd.draw_rectangle(
        roi_x, roi_y, roi_w, roi_h, color=(255, 80, 160, 255), thickness=2
    )

    if SHOW_REJECTED_CIRCLES:
        for candidate in rejected_candidates[0:MAX_DEBUG_CIRCLES]:
            x, y, radius, _score, contrast, saturation, edge_support, reason = (
                candidate
            )
            osd.draw_circle(x, y, radius, color=(255, 255, 0, 0), thickness=2)
            debug_text = "r%d c%d s%d e%d %s" % (
                radius,
                contrast,
                saturation,
                edge_support,
                reason,
            )
            osd.draw_string_advanced(
                max(0, x - radius),
                max(0, y - radius - 18),
                14,
                debug_text,
                color=(255, 255, 0, 0),
            )

    # Tentative tracks are yellow until they have enough temporal evidence.
    pending_tracks = tracker.get_pending_tracks()
    for track in pending_tracks:
        x, y, radius = track.current[0:3]
        osd.draw_circle(x, y, radius, color=(255, 255, 220, 0), thickness=2)
        pending_text = "P%d %d/%d" % (
            track.track_id,
            track.hit_count,
            STABLE_FRAME_COUNT,
        )
        osd.draw_string_advanced(
            max(0, x - radius),
            max(0, y - radius - 17),
            14,
            pending_text,
            color=(255, 255, 220, 0),
        )

    # Every stable ball has its own persistent ID and smoothed coordinates.
    for track in stable_tracks:
        if primary_track is not None and track.track_id == primary_track.track_id:
            continue

        x, y, radius, score = track.current[0:4]
        osd.draw_circle(x, y, radius, color=(255, 0, 255, 255), thickness=3)
        track_text = "ID%d q%d" % (track.track_id, score)
        osd.draw_string_advanced(
            max(0, x - radius),
            max(0, y - radius - 18),
            15,
            track_text,
            color=(255, 0, 255, 255),
        )

    if primary_track is None:
        status = "BALLS:%d  PENDING:%d" % (
            len(stable_tracks),
            len(pending_tracks),
        )
        osd.draw_string_advanced(8, 8, 26, status, color=(255, 255, 210, 0))
    else:
        x, y, radius, score = primary_track.current[0:4]
        osd.draw_circle(x, y, radius, color=(255, 0, 255, 0), thickness=4)
        osd.draw_cross(x, y, color=(255, 0, 255, 0), size=16, thickness=3)
        osd.draw_string_advanced(
            max(0, x - radius),
            max(0, y - radius - 20),
            16,
            "TARGET ID%d" % primary_track.track_id,
            color=(255, 0, 255, 0),
        )
        status = "BALLS:%d TARGET:ID%d x:%d y:%d r:%d" % (
            len(stable_tracks),
            primary_track.track_id,
            x,
            y,
            radius,
        )
        osd.draw_string_advanced(8, 8, 26, status, color=(255, 0, 255, 0))

    osd.draw_cross(
        PICKUP_TARGET_X,
        PICKUP_TARGET_Y,
        color=(255, 255, 0, 255),
        size=8,
        thickness=2,
    )


def main():
    pipeline = None
    uart = None
    frame_number = 0
    last_uart_report_ms = time.ticks_ms() - UART_REPORT_INTERVAL_MS
    tracker = MultiBallTracker()
    primary_track_id = None

    try:
        pipeline = PipeLine(
            rgb888p_size=[FRAME_WIDTH, FRAME_HEIGHT],
            display_size=[FRAME_WIDTH, FRAME_HEIGHT],
            display_mode="lcd",
            osd_layer_num=4,
        )
        pipeline.create(ch1_frame_size=[FRAME_WIDTH, FRAME_HEIGHT])
        uart = YbUart(baudrate=UART_BAUDRATE)

        print("K230 steel-ball detector started")
        print("UART1: 115200 baud, TX=IO9, RX=IO10")

        while True:
            camera_image = pipeline.sensor.snapshot(chn=CAM_CHN_ID_1)
            rgb_image = camera_image.to_rgb888()
            rgb_array = rgb_image.to_numpy_ref()

            candidates, rejected_candidates = find_steel_balls(rgb_array)
            stable_tracks = tracker.update(candidates)
            primary_track = choose_pickup_track(
                stable_tracks, primary_track_id
            )
            if primary_track is None:
                primary_track_id = None
            else:
                primary_track_id = primary_track.track_id

            draw_overlay(
                pipeline.osd_img,
                rejected_candidates,
                tracker,
                stable_tracks,
                primary_track,
            )
            Display.show_image(
                pipeline.osd_img, 0, 0, Display.LAYER_OSD3
            )

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_uart_report_ms) >= UART_REPORT_INTERVAL_MS:
                send_ball_results(uart, stable_tracks)
                last_uart_report_ms = now_ms

            del rgb_array
            del rgb_image
            del camera_image

            frame_number += 1
            if frame_number % GC_INTERVAL_FRAMES == 0:
                gc.collect()

            time.sleep_ms(1)

    except KeyboardInterrupt:
        print("Steel-ball detector stopped by IDE")
    except Exception as error:
        print("Steel-ball detector error:", error)
        raise
    finally:
        if uart is not None:
            try:
                uart.deinit()
            except Exception:
                pass
        if pipeline is not None:
            try:
                pipeline.destroy()
            except Exception:
                pass
        gc.collect()


if __name__ == "__main__":
    main()
