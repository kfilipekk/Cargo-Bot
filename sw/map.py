"""Course map and navigation constants for cargo delivery robot."""

## Course structure
BAYS = [1, 2, 3, 4, 5, 6]
RACKS = ['A', 'B']
LEVELS = ['Upper', 'Lower']

## Side identifiers
RACK_A_SIDE = 'left'
RACK_B_SIDE = 'right'

## Intersection patterns
INTERSECTION_STATE = [1, 1, 1, 1]
T_INTERSECTION_LEFT = [1, 1, 1, 0]
T_INTERSECTION_RIGHT = [0, 1, 1, 1]

## Navigation points
CARGO_SPOT_1_LEFT, CARGO_SPOT_2_LEFT = 1, 2
RACK_A_BAY_1, RACK_A_BAY_2, RACK_A_BAY_3, RACK_A_BAY_4, RACK_A_BAY_5, RACK_A_BAY_6 = 3, 4, 5, 6, 7, 8
CARGO_SPOT_1_RIGHT, CARGO_SPOT_2_RIGHT = 9, 10
RACK_B_BAY_1, RACK_B_BAY_2, RACK_B_BAY_3, RACK_B_BAY_4, RACK_B_BAY_5, RACK_B_BAY_6 = 11, 12, 13, 14, 15, 16

INTERSECTION_MAP = {
    1: 'cargo_1_left', 2: 'cargo_2_left',
    3: 'rack_a_bay_1', 4: 'rack_a_bay_2', 5: 'rack_a_bay_3', 6: 'rack_a_bay_4', 7: 'rack_a_bay_5', 8: 'rack_a_bay_6',
    9: 'cargo_1_right', 10: 'cargo_2_right',
    11: 'rack_b_bay_1', 12: 'rack_b_bay_2', 13: 'rack_b_bay_3', 14: 'rack_b_bay_4', 15: 'rack_b_bay_5', 16: 'rack_b_bay_6',
}

def parse_qr_code(qr_string):
    """Parse QR code: 'Rack A, Upper, 3' -> ('A', 'Upper', 3)"""
    if not qr_string:
        return None, None, None
    try:
        parts = [p.strip() for p in qr_string.split(',')]
        if len(parts) != 3:
            return None, None, None

        rack, level, bay = parts[0].split()[-1].upper(), parts[1].capitalize(), int(parts[2])
        return (rack, level, bay) if rack in RACKS and level in LEVELS and bay in BAYS else (None, None, None)
    except (ValueError, IndexError):
        return None, None, None

def get_bay_intersection(rack, bay_number):
    """Get the intersection number for a given rack and bay."""
    return (RACK_A_BAY_1 if rack == 'A' else RACK_B_BAY_1 if rack == 'B' else None) and \
           (RACK_A_BAY_1 + (bay_number - 1) if rack == 'A' else RACK_B_BAY_1 + (bay_number - 1))

def get_side_from_rack(rack):
    """Get the navigation side from rack identifier."""
    return RACK_A_SIDE if rack == 'A' else RACK_B_SIDE
