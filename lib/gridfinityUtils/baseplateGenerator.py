import math
import adsk.core, adsk.fusion, traceback
import os

from . import const, commonUtils, filletUtils, combineUtils, faceUtils, extrudeUtils, sketchUtils, baseGenerator, patternUtils, shapeUtils, geometryUtils
from .baseGeneratorInput import BaseGeneratorInput
from .baseplateGeneratorInput import BaseplateGeneratorInput

def createGridfinityBaseplate(input: BaseplateGeneratorInput, targetComponent: adsk.fusion.Component):
    features = targetComponent.features
    cutoutInput = BaseGeneratorInput()
    cutoutInput.xyClearance = input.xyClearance
    # Origin at (0, 0) - no offset needed since baseplate is now exact size
    cutoutInput.originPoint = targetComponent.originConstructionPoint.geometry
    # Base dimensions match the actual baseplate dimensions (no clearance addition)
    cutoutInput.baseWidth = input.baseWidth
    cutoutInput.baseLength = input.baseLength
    cutoutInput.cornerFilletRadius = input.cornerFilletRadius + cutoutInput.xyClearance
    baseBody = baseGenerator.createSingleGridfinityBaseBody(cutoutInput, targetComponent)

    cuttingTools: list[adsk.fusion.BRepBody] = [baseBody]
    extraCutoutBodies: list[adsk.fusion.BRepBody] = []

    holeCenterPoint = adsk.core.Point3D.create(
        const.DIMENSION_SCREW_HOLES_OFFSET - input.xyClearance,
        const.DIMENSION_SCREW_HOLES_OFFSET - input.xyClearance,
        0
    )

    connectionHoleYTool = None
    connectionHoleXTool = None

    if input.hasSkeletonizedBottom:
        centerCutoutSketch,centerCutoutSketchCircle = baseGenerator.createCircleAtPointSketch(
            faceUtils.getBottomFace(baseBody),
            input.magnetCutoutsDiameter / 2,
            holeCenterPoint,
            targetComponent
        )
        centerCutoutSketch.name = "center bottom cutout"
        sketchUtils.convertToConstruction(centerCutoutSketch.sketchCurves)
        sketchCurves = centerCutoutSketch.sketchCurves
        dimensions = centerCutoutSketch.sketchDimensions
        constraints = centerCutoutSketch.geometricConstraints
        sketchLines = sketchCurves.sketchLines
        screwHoleCircle = sketchCurves.sketchCircles.item(0)
        arcStartingPoint = screwHoleCircle.centerSketchPoint.geometry.asVector()
        arcStartingPoint.add(adsk.core.Vector3D.create(0, max(input.magnetCutoutsDiameter, input.screwHeadCutoutDiameter) / 2 + const.SKELETON_CLEARANCE, 0))
        arc = sketchCurves.sketchArcs.addByCenterStartSweep(
            screwHoleCircle.centerSketchPoint,
            arcStartingPoint.asPoint(),
            math.radians(90),
        )

        verticalEdgeLine = min([line for line in sketchLines if sketchUtils.isVertical(line)], key=lambda x: abs(x.startSketchPoint.geometry.x))
        horizontalEdgeLine = min([line for line in sketchLines if sketchUtils.isHorizontal(line)], key=lambda x: abs(x.startSketchPoint.geometry.y))

        baseCenterOffsetX = input.baseWidth / 2 - input.xyClearance
        baseCenterOffsetY = input.baseLength / 2 - input.xyClearance
        line1 = sketchLines.addByTwoPoints(arc.startSketchPoint, adsk.core.Point3D.create(verticalEdgeLine.startSketchPoint.geometry.x, arc.startSketchPoint.geometry.y, 0))
        line2 = sketchLines.addByTwoPoints(line1.endSketchPoint, adsk.core.Point3D.create(line1.endSketchPoint.geometry.x, baseCenterOffsetY, 0))
        line3 = sketchLines.addByTwoPoints(line2.endSketchPoint, adsk.core.Point3D.create(-baseCenterOffsetX, baseCenterOffsetY, 0))
        line4 = sketchLines.addByTwoPoints(line3.endSketchPoint, adsk.core.Point3D.create(line3.endSketchPoint.geometry.x, horizontalEdgeLine.startSketchPoint.geometry.y, 0))
        line5 = sketchLines.addByTwoPoints(line4.endSketchPoint, adsk.core.Point3D.create(arc.endSketchPoint.geometry.x, line4.endSketchPoint.geometry.y, 0))
        line6 = sketchLines.addByTwoPoints(line5.endSketchPoint, arc.endSketchPoint)
        
        constraints.addCoincident(line1.endSketchPoint, verticalEdgeLine)
        constraints.addCoincident(line6.startSketchPoint, horizontalEdgeLine)
        constraints.addCoincident(screwHoleCircle.centerSketchPoint, arc.centerSketchPoint)
        constraints.addHorizontal(line1)
        constraints.addPerpendicular(line1, line2)
        constraints.addPerpendicular(line2, line3)
        constraints.addPerpendicular(line3, line4)
        constraints.addPerpendicular(line4, line5)
        constraints.addPerpendicular(line5, line6)
        constraints.addTangent(arc, line1)
        constraints.addEqual(line1, line6)
        constraints.addEqual(line2, line5)
        dimensions.addRadialDimension(arc, arc.endSketchPoint.geometry, True)
        dimensions.addDistanceDimension(
            arc.endSketchPoint,
            line3.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            line2.endSketchPoint.geometry
            )

        centerCutoutExtrudeFeature = extrudeUtils.simpleDistanceExtrude(
            centerCutoutSketch.profiles.item(0),
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
            input.bottomExtensionHeight,
            adsk.fusion.ExtentDirections.PositiveExtentDirection,
            [],
            targetComponent,
        )

        constructionAxisInput: adsk.fusion.ConstructionAxisInput = targetComponent.constructionAxes.createInput()
        constructionAxisInput.setByNormalToFaceAtPoint(
            faceUtils.getBottomFace(baseBody),
            line3.endSketchPoint,
        )
        constructionAxis = targetComponent.constructionAxes.add(constructionAxisInput)
        constructionAxis.isLightBulbOn = False

        centerCutoutPattern = patternUtils.circPattern(
            commonUtils.objectCollectionFromList(centerCutoutExtrudeFeature.bodies),
            constructionAxis,
            4,
            targetComponent,
        )
        centerCutoutBody = centerCutoutExtrudeFeature.bodies.item(0)
        combineUtils.joinBodies(
            centerCutoutBody,
            commonUtils.objectCollectionFromList([body for body in list(centerCutoutPattern.bodies) if not body.name == centerCutoutBody.name]),
            targetComponent,
        )
        extraCutoutBodies.append(centerCutoutBody)
        if input.hasConnectionHoles:
            connectionHoleFaceY = min([face for face in centerCutoutBody.faces if faceUtils.isYNormal(face)], key=lambda x: x.boundingBox.minPoint.y)
            connectionHoleYTool = createConnectionHoleTool(connectionHoleFaceY, input.connectionScrewHolesDiameter / 2, input.baseWidth / 2, targetComponent)
            connectionHoleFaceX = min([face for face in centerCutoutBody.faces if faceUtils.isXNormal(face)], key=lambda x: x.boundingBox.minPoint.x)
            connectionHoleXTool = createConnectionHoleTool(connectionHoleFaceX, input.connectionScrewHolesDiameter / 2, input.baseWidth / 2, targetComponent)

    holeCuttingBodies: list[adsk.fusion.BRepBody] = []
    
    if input.hasExtendedBottom and input.hasMagnetCutouts:
        magnetSocketBody = shapeUtils.simpleCylinder(
            faceUtils.getBottomFace(baseBody),
            0,
            input.magnetCutoutsDepth,
            input.magnetCutoutsDiameter / 2,
            holeCenterPoint,
            targetComponent,
        )
        holeCuttingBodies.append(magnetSocketBody)
    
    if input.hasExtendedBottom and input.hasScrewHoles:
        screwHoleBody = shapeUtils.simpleCylinder(
            faceUtils.getBottomFace(baseBody),
            0,
            input.bottomExtensionHeight,
            input.screwHolesDiameter / 2,
            holeCenterPoint,
            targetComponent,
        )
        holeCuttingBodies.append(screwHoleBody)

        screwHeadHeight = const.DIMENSION_SCREW_HEAD_CUTOUT_OFFSET_HEIGHT + (input.screwHeadCutoutDiameter - input.screwHolesDiameter) / 2
        screwHeadBody = shapeUtils.simpleCylinder(
            faceUtils.getBottomFace(screwHoleBody),
            -screwHeadHeight,
            screwHeadHeight,
            input.screwHeadCutoutDiameter / 2,
            holeCenterPoint,
            targetComponent,
        )
        filletUtils.createChamfer(
            commonUtils.objectCollectionFromList(faceUtils.getTopFace(screwHeadBody).edges),
            (input.screwHeadCutoutDiameter - input.screwHolesDiameter) / 2,
            targetComponent,
        )
        holeCuttingBodies.append(screwHeadBody)

    if len(holeCuttingBodies) > 0:
        patternSpacingX = input.baseWidth - const.DIMENSION_SCREW_HOLES_OFFSET * 2
        patternSpacingY = input.baseLength - const.DIMENSION_SCREW_HOLES_OFFSET * 2
        magnetScrewCutoutsPattern = patternUtils.recPattern(
            commonUtils.objectCollectionFromList(holeCuttingBodies),
            (targetComponent.xConstructionAxis, targetComponent.yConstructionAxis),
            (patternSpacingX, patternSpacingY),
            (2, 2),
            targetComponent
        )
        extraCutoutBodies = extraCutoutBodies + holeCuttingBodies + list(magnetScrewCutoutsPattern.bodies)

    if len(extraCutoutBodies) > 0:
        combineUtils.joinBodies(
            baseBody,
            commonUtils.objectCollectionFromList(extraCutoutBodies),
            targetComponent,
        )
    
    # replicate base in rectangular pattern
    rectangularPatternFeatures: adsk.fusion.RectangularPatternFeatures = features.rectangularPatternFeatures
    patternInputBodies = adsk.core.ObjectCollection.create()
    patternInputBodies.add(baseBody)
    patternInput = rectangularPatternFeatures.createInput(patternInputBodies,
        targetComponent.xConstructionAxis,
        adsk.core.ValueInput.createByReal(input.baseplateWidth),
        adsk.core.ValueInput.createByReal(input.baseWidth),
        adsk.fusion.PatternDistanceType.SpacingPatternDistanceType)
    patternInput.directionTwoEntity = targetComponent.yConstructionAxis
    patternInput.quantityTwo = adsk.core.ValueInput.createByReal(input.baseplateLength)
    patternInput.distanceTwo = adsk.core.ValueInput.createByReal(input.baseLength)
    rectangularPattern = rectangularPatternFeatures.add(patternInput)
    cuttingTools = cuttingTools + list(rectangularPattern.bodies)

    # create baseplate body
    # Baseplate grid dimensions are exactly the grid count × cell size (no clearance subtraction)
    # XY clearance only applies to bin cavity fit, not outer baseplate dimensions
    baseplateTrueWidth = input.baseplateWidth * input.baseWidth
    baseplateTrueLength = input.baseplateLength * input.baseLength
    binInterfaceBody = shapeUtils.simpleBox(
        targetComponent.xYConstructionPlane,
        0,
        input.baseplateWidth * input.baseWidth,
        input.baseplateLength * input.baseLength,
        -const.BIN_BASE_HEIGHT,
        targetComponent.originConstructionPoint.geometry,
        targetComponent,
    )


    if input.hasPadding:
        paddingHeigth = const.BIN_BASE_HEIGHT
        mergeTools = []
        if input.paddingLeft > 0:
            paddingLeftBody = shapeUtils.simpleBox(
                targetComponent.xYConstructionPlane,
                0,
                input.paddingLeft,
                baseplateTrueLength + input.paddingBottom + input.paddingTop,
                -paddingHeigth,
                geometryUtils.createOffsetPoint(
                    targetComponent.originConstructionPoint.geometry,
                    byX=-input.paddingLeft,
                    byY=-input.paddingBottom
                ),
                targetComponent
            )
            paddingLeftBody.name = "Padding left"
            mergeTools.append(paddingLeftBody)
        if input.paddingTop > 0:
            paddingTopBody = shapeUtils.simpleBox(
                targetComponent.xYConstructionPlane,
                0,
                baseplateTrueWidth + input.paddingLeft + input.paddingRight,
                input.paddingTop,
                -paddingHeigth,
                geometryUtils.createOffsetPoint(
                    targetComponent.originConstructionPoint.geometry,
                    byX=-input.paddingLeft,
                    byY=baseplateTrueLength
                ),
                targetComponent
            )
            paddingTopBody.name = "Padding top"
            mergeTools.append(paddingTopBody)
        if input.paddingRight > 0:
            paddingRightBody = shapeUtils.simpleBox(
                targetComponent.xYConstructionPlane,
                0,
                input.paddingRight,
                baseplateTrueLength + input.paddingTop + input.paddingBottom,
                -paddingHeigth,
                geometryUtils.createOffsetPoint(
                    targetComponent.originConstructionPoint.geometry,
                    byX=baseplateTrueWidth,
                    byY=-input.paddingBottom
                ),
                targetComponent
            )
            paddingRightBody.name = "Padding right"
            mergeTools.append(paddingRightBody)
        if input.paddingBottom > 0:
            paddingBottomBody = shapeUtils.simpleBox(
                targetComponent.xYConstructionPlane,
                0,
                baseplateTrueWidth + input.paddingLeft + input.paddingRight,
                input.paddingBottom,
                -paddingHeigth,
                geometryUtils.createOffsetPoint(
                    targetComponent.originConstructionPoint.geometry,
                    byX=-input.paddingLeft,
                    byY=-input.paddingBottom
                ),
                targetComponent
            )
            paddingBottomBody.name = "Padding bottom"
            mergeTools.append(paddingBottomBody)
        if len(mergeTools) > 0:
            paddingCombineFeature = combineUtils.joinBodies(
                binInterfaceBody,
                commonUtils.objectCollectionFromList(mergeTools),
                targetComponent,
            )
            paddingCombineFeature.name = "Combine base with padding bodies"
            binInterfaceBody = paddingCombineFeature.bodies.item(0)

    cornerFillet = filletUtils.filletEdgesByLength(
        binInterfaceBody.faces,
        input.cornerFilletRadius - input.xyClearance,
        const.BIN_BASE_HEIGHT,
        targetComponent,
        )
    cornerFillet.name = "Round outer corners"
    
    if input.hasExtendedBottom:
        baseplateBottomLayer = extrudeUtils.simpleDistanceExtrude(
            faceUtils.getBottomFace(binInterfaceBody),
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
            input.bottomExtensionHeight,
            adsk.fusion.ExtentDirections.PositiveExtentDirection,
            [],
            targetComponent,
        )
        baseplateBottomLayerBody = baseplateBottomLayer.bodies.item(0)
        combineUtils.joinBodies(binInterfaceBody, commonUtils.objectCollectionFromList([baseplateBottomLayerBody]), targetComponent)

    bottomChamfer = filletUtils.chamferEdgesByLength(
        [faceUtils.getBottomFace(binInterfaceBody)],
        const.BASEPLATE_BOTTOM_CHAMFER_LENGTH,
        baseplateTrueLength + (input.paddingTop + input.paddingBottom if input.hasPadding else 0),
        const.BIN_CORNER_FILLET_RADIUS * 3,
        targetComponent,
    )
    bottomChamfer.name = "Bottom chamfer"

    if not connectionHoleYTool is None and not connectionHoleXTool is None:
        holeToolsXFeature = patternUtils.recPattern(
            commonUtils.objectCollectionFromList(connectionHoleXTool.bodies),
            (targetComponent.xConstructionAxis, targetComponent.yConstructionAxis),
            (input.baseWidth, input.baseLength),
            (1, input.baseplateLength),
            targetComponent
        )
        connectionHoleXToolList = list(connectionHoleXTool.bodies) + list(holeToolsXFeature.bodies)

        holeToolsYFeature = patternUtils.recPattern(
            commonUtils.objectCollectionFromList(connectionHoleYTool.bodies),
            (targetComponent.xConstructionAxis, targetComponent.yConstructionAxis),
            (input.baseLength, input.baseLength),
            (input.baseplateWidth, 1),
            targetComponent
        )
        connectionHoleYToolList = list(connectionHoleYTool.bodies) + list(holeToolsYFeature.bodies)

        constructionPlaneXZInput: adsk.fusion.ConstructionPlaneInput = targetComponent.constructionPlanes.createInput()
        constructionPlaneXZInput.setByOffset(targetComponent.xZConstructionPlane, adsk.core.ValueInput.createByReal(input.baseplateLength * input.baseLength / 2 - input.xyClearance))
        constructionPlaneXZ = targetComponent.constructionPlanes.add(constructionPlaneXZInput)
        constructionPlaneXZ.isLightBulbOn = False

        constructionPlaneYZInput: adsk.fusion.ConstructionPlaneInput = targetComponent.constructionPlanes.createInput()
        constructionPlaneYZInput.setByOffset(targetComponent.yZConstructionPlane, adsk.core.ValueInput.createByReal(input.baseplateWidth * input.baseWidth / 2 - input.xyClearance))
        constructionPlaneYZ = targetComponent.constructionPlanes.add(constructionPlaneYZInput)
        constructionPlaneYZ.isLightBulbOn = False

        mirrorConnectionHolesYZInput = features.mirrorFeatures.createInput(commonUtils.objectCollectionFromList(connectionHoleXToolList), constructionPlaneYZ)
        mirrorConnectionHolesYZ = features.mirrorFeatures.add(mirrorConnectionHolesYZInput)

        mirrorConnectionHolesXZInput = features.mirrorFeatures.createInput(commonUtils.objectCollectionFromList(connectionHoleYToolList), constructionPlaneXZ)
        mirrorConnectionHolesXZ = features.mirrorFeatures.add(mirrorConnectionHolesXZInput)

        cuttingTools = cuttingTools + list(mirrorConnectionHolesYZ.bodies) + list(mirrorConnectionHolesXZ.bodies) + connectionHoleYToolList + connectionHoleXToolList


    # cut everything
    toolBodies = commonUtils.objectCollectionFromList(cuttingTools)
    finalCut = combineUtils.cutBody(
        binInterfaceBody,
        toolBodies,
        targetComponent,
    )
    finalCut.name = "Final baseplate cut"

    # Create clip cutouts (if enabled)
    createClipCutouts(input, binInterfaceBody, targetComponent)

    return binInterfaceBody

def createConnectionHoleTool(connectionHoleFace: adsk.fusion.BRepFace, diameter: float, depth: float, targetComponent: adsk.fusion.Component):
    connectionHoleSketch: adsk.fusion.Sketch = targetComponent.sketches.add(connectionHoleFace)
    connectionHoleSketch.name = "side connector hole"
    sketchCurves = connectionHoleSketch.sketchCurves
    dimensions = connectionHoleSketch.sketchDimensions
    constraints = connectionHoleSketch.geometricConstraints
    sketchUtils.convertToConstruction(sketchCurves)
    [sketchHorizontalEdge1, sketchHorizontalEdge2] = [line for line in sketchCurves.sketchLines if sketchUtils.isHorizontal(line)]
    line1 = sketchCurves.sketchLines.addByTwoPoints(sketchHorizontalEdge1.startSketchPoint.geometry, sketchHorizontalEdge2.endSketchPoint.geometry)
    line1.isConstruction = True
    constraints.addMidPoint(line1.startSketchPoint, sketchHorizontalEdge1)
    constraints.addMidPoint(line1.endSketchPoint, sketchHorizontalEdge2)
    
    circle = sketchCurves.sketchCircles.addByCenterRadius(
        connectionHoleSketch.originPoint.geometry,
        diameter
    )
    constraints.addMidPoint(circle.centerSketchPoint, line1)
    dimensions.addRadialDimension(circle, line1.startSketchPoint.geometry, True)
    connectionHoleTool = extrudeUtils.simpleDistanceExtrude(
        connectionHoleSketch.profiles.item(0),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
        depth,
        adsk.fusion.ExtentDirections.PositiveExtentDirection,
        [],
        targetComponent,
    )
    return connectionHoleTool

def createClipCutoutBodies(
    originPoint: adsk.core.Point3D,
    sketchPlane,
    targetComponent: adsk.fusion.Component,
    edgeAxis: str = "X",  # "X" for left/right edges, "Y" for top/bottom edges
    isRightEdge: bool = False,  # True for right edge (mirror in X)
    isTopEdge: bool = False,  # True for top edge, False for bottom edge (only used for Y-axis)
    clipNumber: int = 1   # Clip number for naming
):
    """
    Create the 4-part stepped clip cutout profile STACKED VERTICALLY IN Z:
    Part 1 (Rectangle): 21.5mm wide × 19.8mm length × 26.5mm deep (deepest)
    Part 2 (Rectangle): 8.5mm wide (offset 1.3mm) × 19.8mm length × 5mm deep
    Part 3 (Rectangle): 10mm wide (offset 1.0mm) × 19.8mm length × 4mm deep
    Part 4 (Rectangle): 11.5mm wide (offset 1.3mm) × 19.8mm length × 4mm deep (shallowest)

    All 4 parts start at the same XY position and stack vertically (Z direction) with different depths.

    Args:
        originPoint: Origin point for the cutout profile (at the grid edge)
        sketchPlane: A clean construction plane to create sketches on (avoids inherited geometry issues)
        targetComponent: The component to create the bodies in
        edgeAxis: "X" for vertical edges (left/right), "Y" for horizontal edges (top/bottom)
        isRightEdge: True if right edge (X = max), False if left edge (X = 0)
        clipNumber: Clip number for naming (used in sketch and feature names)

    Returns:
        List of BRepBody objects to be cut from the baseplate
    """
    from ...lib import fusion360utils as futil

    cutoutBodies = []

    # Helper function to create a rectangle sketch and extrude it
    def createAndExtrudePart(partNum, partName, width, edgeInset, depth, zInset, taperAngle=0):
        """
        Create a rectangular profile and extrude it to create a body.

        Args:
            partNum: Part number (1-4)
            partName: Name of the part (e.g., "rectangle" or "trapezoid")
            width: Width of the part perpendicular to the edge
            edgeInset: Inset from the outer edge in XY plane
            depth: Extrusion depth into the baseplate
            zInset: Z offset (vertical offset from top surface where extrusion starts)
            taperAngle: Optional taper angle in degrees (0 for no taper, non-zero creates tapered shape)
        """
        # Create sketch on the clean construction plane
        sketch = targetComponent.sketches.add(sketchPlane)
        sketch.name = f"sketch - cutout {clipNumber} - part {partNum} - {partName}"
        sketchLines = sketch.sketchCurves.sketchLines

        # Convert origin point to THIS SPECIFIC SKETCH's coordinate system
        originInSketch = sketch.modelToSketchSpace(originPoint)

        if edgeAxis == "X":
            # Left/right edges: profile extends in X direction, length along Y direction
            if isRightEdge:
                # Right edge: mirror the X coordinates (extend backward from right edge)
                p1 = adsk.core.Point3D.create(originInSketch.x - edgeInset, originInSketch.y, originInSketch.z)
                p2 = adsk.core.Point3D.create(originInSketch.x - edgeInset - width, originInSketch.y, originInSketch.z)
                p3 = adsk.core.Point3D.create(originInSketch.x - edgeInset - width, originInSketch.y + const.CLIP_PROFILE_LENGTH, originInSketch.z)
                p4 = adsk.core.Point3D.create(originInSketch.x - edgeInset, originInSketch.y + const.CLIP_PROFILE_LENGTH, originInSketch.z)
            else:
                # Left edge: standard orientation
                p1 = adsk.core.Point3D.create(originInSketch.x + edgeInset, originInSketch.y, originInSketch.z)
                p2 = adsk.core.Point3D.create(originInSketch.x + edgeInset + width, originInSketch.y, originInSketch.z)
                p3 = adsk.core.Point3D.create(originInSketch.x + edgeInset + width, originInSketch.y + const.CLIP_PROFILE_LENGTH, originInSketch.z)
                p4 = adsk.core.Point3D.create(originInSketch.x + edgeInset, originInSketch.y + const.CLIP_PROFILE_LENGTH, originInSketch.z)
        else:
            # Top/bottom edges: profile extends in Y direction, length along X direction
            if isTopEdge:
                # Top edge: extend downward from origin (negative Y direction)
                p1 = adsk.core.Point3D.create(originInSketch.x, originInSketch.y - edgeInset, originInSketch.z)
                p2 = adsk.core.Point3D.create(originInSketch.x + const.CLIP_PROFILE_LENGTH, originInSketch.y - edgeInset, originInSketch.z)
                p3 = adsk.core.Point3D.create(originInSketch.x + const.CLIP_PROFILE_LENGTH, originInSketch.y - edgeInset - width, originInSketch.z)
                p4 = adsk.core.Point3D.create(originInSketch.x, originInSketch.y - edgeInset - width, originInSketch.z)
            else:
                # Bottom edge: extend upward from origin (positive Y direction)
                p1 = adsk.core.Point3D.create(originInSketch.x, originInSketch.y + edgeInset, originInSketch.z)
                p2 = adsk.core.Point3D.create(originInSketch.x + const.CLIP_PROFILE_LENGTH, originInSketch.y + edgeInset, originInSketch.z)
                p3 = adsk.core.Point3D.create(originInSketch.x + const.CLIP_PROFILE_LENGTH, originInSketch.y + edgeInset + width, originInSketch.z)
                p4 = adsk.core.Point3D.create(originInSketch.x, originInSketch.y + edgeInset + width, originInSketch.z)

        sketchLines.addByTwoPoints(p1, p2)
        sketchLines.addByTwoPoints(p2, p3)
        sketchLines.addByTwoPoints(p3, p4)
        sketchLines.addByTwoPoints(p4, p1)

        if sketch.profiles.count > 0:
            try:
                # Use profile 0 - it's the only profile since sketch is on clean construction plane
                feature = extrudeUtils.simpleDistanceExtrude(
                    sketch.profiles.item(0),
                    adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                    depth,
                    adsk.fusion.ExtentDirections.NegativeExtentDirection,
                    [],
                    targetComponent,
                    startOffset=zInset,
                    taperAngle=taperAngle
                )
                feature.name = f"extrude - cutout {clipNumber} - part {partNum} - {partName}"
                body = feature.bodies.item(0)
                body.name = f"cutout {clipNumber} - part {partNum} - {partName}"
                cutoutBodies.append(body)
            except Exception as e:
                futil.log(f'createClipCutoutBodies: ERROR extruding part {partNum}: {str(e)}')

    # All parts start at the same XY position, stacked vertically in Z
    # Part 1: Full width 21.5mm, starts at Z=0 (deepest)
    createAndExtrudePart(1, "rectangle", const.CLIP_PART1_WIDTH, const.CLIP_PART1_EDGE_INSET, const.CLIP_PART1_DEPTH, const.CLIP_PART1_Z_INSET)

    # Part 2: 8.5mm wide, starts at Z=2.65mm
    createAndExtrudePart(2, "rectangle", const.CLIP_PART2_WIDTH, const.CLIP_PART2_EDGE_INSET, const.CLIP_PART2_DEPTH, const.CLIP_PART2_Z_INSET)
    part2Body = cutoutBodies[-1]

    # Part 3: Loft from bottom of Part 2 to top of Part 4
    # Create Part 4 first so we have its face to loft to
    createAndExtrudePart(4, "rectangle", const.CLIP_PART4_WIDTH, const.CLIP_PART4_EDGE_INSET, const.CLIP_PART4_DEPTH, const.CLIP_PART4_Z_INSET)
    part4Body = cutoutBodies[-1]

    # Now create loft between Part 2's bottom face and Part 4's top face
    try:

        # Helper function to check if a face is horizontal (normal parallel to Z-axis)
        def isHorizontalFace(face: adsk.fusion.BRepFace) -> bool:
            """Check if a face's normal is approximately parallel to the Z-axis."""
            try:
                # Get the surface geometry of the face
                surfaceGeom = face.geometry
                # For planar faces, check the normal direction
                if hasattr(surfaceGeom, 'normal'):
                    normal = surfaceGeom.normal
                    # A horizontal face has normal (0, 0, 1) or (0, 0, -1)
                    # Check if X and Y components are near zero (tolerance 0.01)
                    return abs(normal.x) < 0.01 and abs(normal.y) < 0.01
                return False
            except:
                return False

        # Find the bottom face of Part 2 (horizontal face with lowest Z coordinate)
        part2BottomFace = None
        part2MinZ = float('inf')
        for face in part2Body.faces:
            try:
                # Only consider horizontal faces (normal parallel to Z-axis)
                if not isHorizontalFace(face):
                    continue
                # Get the bounding box of the face to determine its Z position
                boundingBox = face.boundingBox
                # For horizontal faces, minPoint.z == maxPoint.z (within tolerance)
                faceZ = boundingBox.minPoint.z
                if faceZ < part2MinZ:
                    part2MinZ = faceZ
                    part2BottomFace = face
            except:
                pass

        # Find the top face of Part 4 (horizontal face with highest Z coordinate)
        part4TopFace = None
        part4MaxZ = float('-inf')
        for face in part4Body.faces:
            try:
                # Only consider horizontal faces (normal parallel to Z-axis)
                if not isHorizontalFace(face):
                    continue
                # Get the bounding box of the face to determine its Z position
                boundingBox = face.boundingBox
                # For horizontal faces, minPoint.z == maxPoint.z (within tolerance)
                faceZ = boundingBox.maxPoint.z
                if faceZ > part4MaxZ:
                    part4MaxZ = faceZ
                    part4TopFace = face
            except:
                pass

        if part2BottomFace and part4TopFace:
            try:
                # Create loft feature
                loftFeatures = targetComponent.features.loftFeatures
                loftInput = loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

                # Add the two profiles (faces) to loft between
                loftInput.loftSections.add(part2BottomFace)
                loftInput.loftSections.add(part4TopFace)

                # Create the loft
                loftFeature = loftFeatures.add(loftInput)
                loftFeature.name = f"loft - cutout {clipNumber} - part 3 - transition"

                # Get the body created by the loft
                if loftFeature.bodies.count > 0:
                    loftBody = loftFeature.bodies.item(0)
                    loftBody.name = f"cutout {clipNumber} - part 3 - transition"
                    cutoutBodies.insert(len(cutoutBodies) - 1, loftBody)  # Insert before Part 4
                else:
                    futil.log(f'createClipCutoutBodies: WARNING - Loft feature has no bodies')
            except Exception as loftError:
                futil.log(f'createClipCutoutBodies: ERROR in loft creation: {str(loftError)}')
        else:
            futil.log(f'createClipCutoutBodies: Could not find appropriate faces for loft (part2 bottom: {part2BottomFace is not None}, part4 top: {part4TopFace is not None})')
    except Exception as e:
        futil.log(f'createClipCutoutBodies: ERROR creating loft for part 3: {str(e)}')

    return cutoutBodies

def createClipCutouts(
    input: BaseplateGeneratorInput,
    targetPlateBody: adsk.fusion.BRepBody,
    targetComponent: adsk.fusion.Component,
):
    """
    Create clip cutouts along the selected edges of the baseplate.
    Each clip is positioned at the center of a grid cell.
    """
    from ...lib import fusion360utils as futil

    if not input.hasClips:
        return

    clipCuttingBodies = []

    # Calculate baseplate dimensions
    baseWidth = input.baseWidth  # Width of one grid cell
    baseLength = input.baseLength  # Length of one grid cell
    gridCountX = input.baseplateWidth  # Number of grids in X direction
    gridCountY = input.baseplateLength  # Number of grids in Y direction

    # Clip positioning uses exact grid dimensions (no clearance subtraction)
    # Clips should align with grid edges at 0 and width/length
    baseplateTrueWidth = input.baseplateWidth * baseWidth
    baseplateTrueLength = input.baseplateLength * baseLength

    # Create clip cutouts as rectangular sketches on a clean construction plane
    # Get the top face of the baseplate to determine the plane level
    topFace = faceUtils.getTopFace(targetPlateBody)

    # Create a CLEAN construction plane at the top face level (no inherited geometry)
    # This avoids coordinate system issues from inherited sketches on topFace
    constructionPlaneInput = targetComponent.constructionPlanes.createInput()
    constructionPlaneInput.setByOffset(targetComponent.xYConstructionPlane, adsk.core.ValueInput.createByReal(0))
    clipSketchPlane = targetComponent.constructionPlanes.add(constructionPlaneInput)
    clipSketchPlane.name = "Clip sketch plane"

    # Left edge clips (X = 0, Y varies)
    if input.hasClipsLeft:
        for i in range(gridCountY):
            clipCenterY = i * baseLength + (baseLength / 2)
            # Position at left edge (X=0), centered on grid cell
            clipOrigin = geometryUtils.createOffsetPoint(
                targetComponent.originConstructionPoint.geometry,
                byX=0,
                byY=clipCenterY - (const.CLIP_PROFILE_LENGTH / 2),
                byZ=0
            )

            # Create the cutout bodies using decomposed rectangles
            clipBodies = createClipCutoutBodies(clipOrigin, clipSketchPlane, targetComponent, edgeAxis="X", clipNumber=len(clipCuttingBodies) + 1)
            clipCuttingBodies.extend(clipBodies)

    # Right edge clips (X = max, Y varies)
    if input.hasClipsRight:
        for i in range(gridCountY):
            clipCenterY = i * baseLength + (baseLength / 2)
            # Position at right edge (X=max), centered on grid cell
            clipOrigin = geometryUtils.createOffsetPoint(
                targetComponent.originConstructionPoint.geometry,
                byX=baseplateTrueWidth,
                byY=clipCenterY - (const.CLIP_PROFILE_LENGTH / 2),
                byZ=0
            )

            # Create the cutout bodies using decomposed rectangles
            clipBodies = createClipCutoutBodies(clipOrigin, clipSketchPlane, targetComponent, edgeAxis="X", isRightEdge=True, clipNumber=len(clipCuttingBodies) + 1)
            clipCuttingBodies.extend(clipBodies)

    # Bottom edge clips (Y = 0, X varies)
    if input.hasClipsBottom:
        for i in range(gridCountX):
            clipCenterX = i * baseWidth + (baseWidth / 2)
            # Position at bottom edge (Y=0), centered on grid cell
            clipOrigin = geometryUtils.createOffsetPoint(
                targetComponent.originConstructionPoint.geometry,
                byX=clipCenterX - (const.CLIP_PROFILE_LENGTH / 2),
                byY=0,
                byZ=0
            )

            # Create the cutout bodies using decomposed rectangles
            clipBodies = createClipCutoutBodies(clipOrigin, clipSketchPlane, targetComponent, edgeAxis="Y", clipNumber=len(clipCuttingBodies) + 1)
            clipCuttingBodies.extend(clipBodies)

    # Top edge clips (Y = max, X varies)
    if input.hasClipsTop:
        for i in range(gridCountX):
            clipCenterX = i * baseWidth + (baseWidth / 2)
            # Position at top edge (Y=max), centered on grid cell
            clipOrigin = geometryUtils.createOffsetPoint(
                targetComponent.originConstructionPoint.geometry,
                byX=clipCenterX - (const.CLIP_PROFILE_LENGTH / 2),
                byY=baseplateTrueLength,
                byZ=0
            )

            # Create the cutout bodies using decomposed rectangles
            clipBodies = createClipCutoutBodies(clipOrigin, clipSketchPlane, targetComponent, edgeAxis="Y", isTopEdge=True, clipNumber=len(clipCuttingBodies) + 1)
            clipCuttingBodies.extend(clipBodies)

    # Cut all clip bodies from the baseplate
    if clipCuttingBodies:
        try:
            clipCuttingCollection = commonUtils.objectCollectionFromList(clipCuttingBodies)
            combineUtils.cutBody(targetPlateBody, clipCuttingCollection, targetComponent)
        except Exception as e:
            futil.log(f'createClipCutouts: ERROR during cut operation: {type(e).__name__}: {str(e)}')
            raise