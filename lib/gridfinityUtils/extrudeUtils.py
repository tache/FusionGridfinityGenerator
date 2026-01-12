import adsk.core, adsk.fusion, traceback
import os

from . import sketchUtils

def simpleDistanceExtrude(
    profile: adsk.core.Base,
    operation: adsk.fusion.FeatureOperations,
    distance: float,
    direction: adsk.fusion.ExtentDirections,
    participantBodies: list[adsk.fusion.BRepBody],
    targetComponent: adsk.fusion.Component,
    startOffset: float = 0,
    taperAngle: float = 0,
    ):
    features: adsk.fusion.Features = targetComponent.features
    extrudeFeatures: adsk.fusion.ExtrudeFeatures = features.extrudeFeatures
    extrudeInput = extrudeFeatures.createInput(profile, operation)
    extrudeInput.participantBodies = participantBodies

    # Set the start offset if provided
    if startOffset != 0:
        startOffsetInput = adsk.core.ValueInput.createByReal(startOffset)
        startOffsetDef = adsk.fusion.OffsetStartDefinition.create(startOffsetInput)
        extrudeInput.startExtent = startOffsetDef

    # Define the extrusion extent from the start plane (or sketch plane if no offset)
    extrudeExtent = adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(distance))

    # Set the one-side extent with optional taper angle
    if taperAngle != 0:
        extrudeInput.setOneSideExtent(extrudeExtent, direction, adsk.core.ValueInput.createByReal(taperAngle))
    else:
        extrudeInput.setOneSideExtent(extrudeExtent, direction)

    extrudeFeature = extrudeFeatures.add(extrudeInput)
    return extrudeFeature

def createBox(
    width: float,
    length: float,
    height: float,
    targetComponent: adsk.fusion.Component,
    targetPlane: adsk.core.Base,
    ):
    features: adsk.fusion.Features = targetComponent.features
    extrudeFeatures: adsk.fusion.ExtrudeFeatures = features.extrudeFeatures
    sketches: adsk.fusion.Sketches = targetComponent.sketches
    recSketch: adsk.fusion.Sketch = sketches.add(targetPlane)
    recSketch.name = 'Simple box sketch'
    sketchUtils.createRectangle(width, length, recSketch.originPoint.geometry, recSketch)

    # extrude
    extrude = extrudeFeatures.addSimple(recSketch.profiles.item(0),
        adsk.core.ValueInput.createByReal(height),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extrude.name = 'Simple box extrude'
    return extrude

def createBoxAtPoint(
    width: float,
    length: float,
    height: float,
    targetComponent: adsk.fusion.Component,
    originPoint: adsk.core.Point3D,
    ):
    features: adsk.fusion.Features = targetComponent.features
    extrudeFeatures: adsk.fusion.ExtrudeFeatures = features.extrudeFeatures
    boxPlaneInput: adsk.fusion.ConstructionPlaneInput = targetComponent.constructionPlanes.createInput()
    boxPlaneInput.setByOffset(
        targetComponent.xYConstructionPlane,
        adsk.core.ValueInput.createByReal(originPoint.z)
    )
    boxConstructionPlane = targetComponent.constructionPlanes.add(boxPlaneInput)
    boxConstructionPlane.name = 'Simple box at point construction plane'
    sketches: adsk.fusion.Sketches = targetComponent.sketches
    recSketch: adsk.fusion.Sketch = sketches.add(boxConstructionPlane)
    recSketch.name = 'Simple box at point sketch'
    sketchUtils.createRectangle(width, length, recSketch.modelToSketchSpace(originPoint), recSketch)
        
    # extrude
    extrude = extrudeFeatures.addSimple(recSketch.profiles.item(0),
        adsk.core.ValueInput.createByReal(height),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extrude.name = 'Simple box at point extrude'
    return extrude
