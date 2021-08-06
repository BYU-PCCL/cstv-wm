from typing import Callable

from .types import (
    ExtendedWMNormalHints,
    WM_NORMAL_HINTS_FIELD_BITMASK_MAP,
    WindowGeometry,
)


def debug_log_size_hints(
    log_fn: Callable[[str], None], size_hints: ExtendedWMNormalHints
):
    flags = size_hints.flags
    for field_type, field_map in WM_NORMAL_HINTS_FIELD_BITMASK_MAP.items():
        masked_fields = []
        for mask, fields in field_map.items():
            if not flags & mask:
                continue

            masked_fields += fields

        if not len(masked_fields):
            continue

        log_fn(f"- {field_type}:")
        [
            log_fn(f"  - {field}: {getattr(size_hints, field)}")
            for field in masked_fields
        ]


def debug_log_window_geometry(log_fn: Callable[[str], None], geometry: WindowGeometry):
    log_fn(f"- x: {geometry.x}")
    log_fn(f"- y: {geometry.y}")
    log_fn(f"- width: {geometry.width}")
    log_fn(f"- height: {geometry.height}")


def debug_value_change(log_fn: Callable[[str], None], name: str, old, new):
    if old != new:
        log_fn(f"- {name}:")
        log_fn(f"  - old: {old}")
        log_fn(f"  - new: {new}")
    else:
        log_fn(f"- {name} unchanged: {new}")
