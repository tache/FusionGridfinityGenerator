import adsk.core, adsk.fusion, traceback

BIN_XY_CLEARANCE = 0.025
BIN_CORNER_FILLET_RADIUS = 0.4

BIN_BASE_TOP_SECTION_HEIGH = 0.215
BIN_BASE_MID_SECTION_HEIGH = 0.18
BIN_BASE_BOTTOM_SECTION_HEIGH = 0.07
BIN_BASE_HEIGHT = BIN_BASE_TOP_SECTION_HEIGH + BIN_BASE_MID_SECTION_HEIGH + BIN_BASE_BOTTOM_SECTION_HEIGH
BIN_MAGNET_HOLE_GROOVE_DEPTH = 0.06

BIN_LIP_EXTRA_HEIGHT = 0.44
BIN_LIP_WALL_THICKNESS = BIN_BASE_TOP_SECTION_HEIGH - BIN_XY_CLEARANCE
BIN_LIP_TOP_RECESS_HEIGHT = 0.06
BIN_WALL_THICKNESS = 0.12
BIN_CONNECTION_RECESS_DEPTH = 0.44
BIN_COMPARTMENT_BOTTOM_THICKNESS = 0.1
BIN_BODY_CUTOUT_BOTTOM_FILLET_RADIUS = 0.2

BIN_TAB_WIDTH = 1.3
BIN_TAB_OVERHANG_ANGLE = 45
BIN_TAB_LABEL_ANGLE = 0
BIN_TAB_EDGE_FILLET_RADIUS = 0.06
BIN_TAB_TOP_CLEARANCE = 0.05

BIN_SCOOP_MAX_RADIUS = 2.5

BASEPLATE_EXTRA_HEIGHT = 0.64
BASEPLATE_BIN_Z_CLEARANCE = 0.05

DIMENSION_DEFAULT_WIDTH_UNIT = 4.2
DIMENSION_DEFAULT_HEIGHT_UNIT = 0.7
DIMENSION_SCREW_HOLES_OFFSET = 0.8
DIMENSION_SCREW_HOLE_DIAMETER = 0.3
DIMENSION_PLATE_CONNECTION_SCREW_HOLE_DIAMETER = 0.32
DIMENSION_PLATE_SCREW_HOLE_DIAMETER = 0.32
DIMENSION_SCREW_HEAD_CUTOUT_DIAMETER = 0.6
DIMENSION_SCREW_HEAD_CUTOUT_OFFSET_HEIGHT = 0.05
DIMENSION_MAGNET_CUTOUT_DIAMETER = 0.65
DIMENSION_MAGNET_CUTOUT_DEPTH = 0.24
DIMENSION_PRINT_HELPER_GROOVE_DEPTH = 0.03

# Clip connector dimensions (from STL analysis)
# Connector: 4.3mm thick × 3.67mm tall × 19.6mm long
CLIP_CONNECTOR_THICKNESS = 0.43  # 4.3mm - how thick the connector is
CLIP_CONNECTOR_HEIGHT = 0.367  # 3.67mm - vertical height (Z-axis)
CLIP_CONNECTOR_LENGTH = 1.96  # 19.6mm - length along the edge
CLIP_CUTOUT_CLEARANCE = 0.01  # 0.1mm per side = 0.2mm total
CLIP_CUTOUT_EDGE_OVERSHOOT = 0.01  # 0.1mm overshoot to ensure clean cut at edges
CLIP_CUTOUT_DEPTH_INTO_EDGE = (CLIP_CONNECTOR_THICKNESS / 2) + CLIP_CUTOUT_CLEARANCE  # Half thickness (2.25mm)
CLIP_CUTOUT_HEIGHT = CLIP_CONNECTOR_HEIGHT + CLIP_CUTOUT_CLEARANCE  # Vertical (3.77mm)
CLIP_CUTOUT_LENGTH = CLIP_CONNECTOR_LENGTH + CLIP_CUTOUT_CLEARANCE  # Along edge (19.7mm)

# Baseplate clip cutout profile specifications
# All 4 parts are stacked vertically in Z (nested rectangles)
# All parts have the same length: 19.8mm (along the edge)
# XY dimensions: parts are positioned at the edge (0mm inset for Part 1)
# Z dimensions: parts are stacked with Z insets specifying where each starts from top surface

# Part 1: Full width base rectangle (outermost)
CLIP_PART1_WIDTH = 0.215      # Width in cm
CLIP_PART1_DEPTH = 0.265      # Extrusion depth in cm
CLIP_PART1_EDGE_INSET = 0.0   # Edge inset from outer edge in cm (0.0mm)
CLIP_PART1_Z_INSET = 0.0      # Z offset from top surface in cm (0.0mm)

# Part 2: Middle rectangle (narrower, nested inside Part 1)
CLIP_PART2_WIDTH = 0.085      # Width in cm
CLIP_PART2_DEPTH = 0.050      # Extrusion depth in cm
CLIP_PART2_EDGE_INSET = 0.13  # Edge inset from outer edge in cm (13mm) - aligns inner edge with Part 1
CLIP_PART2_Z_INSET = -0.265   # Z offset from top surface in cm (2.65mm)

# Part 3: Lofted transition section (created via loft between Part 2 and Part 4)
# No direct constants needed - loft uses the faces of Part 2 and Part 4 to create the transition

# Part 4: Top rectangle (innermost, shallowest)
CLIP_PART4_WIDTH = 0.115      # Width in cm
CLIP_PART4_DEPTH = 0.040      # Extrusion depth in cm
CLIP_PART4_EDGE_INSET = 0.10  # Edge inset from outer edge in cm (10mm) - aligns inner edge with Part 1
CLIP_PART4_Z_INSET = -0.355   # Z offset from top surface in cm (3.55mm)

# Common clip dimension
CLIP_PROFILE_LENGTH = 1.96   # Length along edge in mm (same for all parts)

# Baseplate-specific specifications
BASEPLATE_BOTTOM_CHAMFER_LENGTH = 0.005  # Chamfer length in mm
SKELETON_CLEARANCE = 0.1     # Clearance margin in mm for skeleton structure

DEFAULT_FILTER_TOLERANCE = 0.00001