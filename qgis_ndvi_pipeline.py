"""
QGIS-side NDVI processing pipeline.

Run this from the QGIS Python console, or headless via:
    qgis_process --python qgis_ndvi_pipeline.py
    (or) python3 qgis_ndvi_pipeline.py   # with PyQGIS on PYTHONPATH

Pipeline:
  1. Load mosaic.tif (written by ndvi_geotiff_export.py)
  2. Style it with a red->yellow->green ramp for the live map view
  3. Reclassify: NDVI < STRESS_THRESHOLD -> 1 (spray), else -> 0 (skip)
  4. Polygonize the "spray" raster into a vector layer
  5. Extract polygon centroids -> spray_targets.geojson
     (lat, lon, mean_ndvi, area_m2 per target — this is what the flight
      planner / Jetson decision engine consumes to build the spray route)
"""

import processing
from qgis.core import (
    QgsProject, QgsRasterLayer, QgsVectorLayer, QgsSingleBandPseudoColorRenderer,
    QgsColorRampShader, QgsRasterShader, QgsApplication,
)

MOSAIC_PATH = "mosaic.tif"
STRESS_THRESHOLD = 0.35
RECLASS_PATH = "stress_mask.tif"
POLYGON_PATH = "stress_zones.gpkg"
TARGETS_PATH = "spray_targets.geojson"


def style_ndvi_layer(layer: QgsRasterLayer):
    shader = QgsRasterShader()
    color_ramp = QgsColorRampShader()
    color_ramp.setColorRampType(QgsColorRampShader.Interpolated)
    color_ramp.setColorRampItemList([
        QgsColorRampShader.ColorRampItem(-1.0, __import__("qgis.PyQt.QtGui", fromlist=["QColor"]).QColor(165, 0, 38)),
        QgsColorRampShader.ColorRampItem(0.35, __import__("qgis.PyQt.QtGui", fromlist=["QColor"]).QColor(255, 255, 191)),
        QgsColorRampShader.ColorRampItem(1.0, __import__("qgis.PyQt.QtGui", fromlist=["QColor"]).QColor(0, 104, 55)),
    ])
    shader.setRasterShaderFunction(color_ramp)
    renderer = QgsSingleBandPseudoColorRenderer(layer.dataProvider(), 1, shader)
    layer.setRenderer(renderer)
    layer.triggerRepaint()


def reclassify_stress(mosaic_path: str) -> str:
    """NDVI < STRESS_THRESHOLD -> 1, else -> 0, via the raster calculator."""
    result = processing.run("gdal:rastercalculator", {
        "INPUT_A": mosaic_path,
        "BAND_A": 1,
        "FORMULA": f"(A<{STRESS_THRESHOLD})*1",
        "OUTPUT": RECLASS_PATH,
        "RTYPE": 0,  # Byte
        "NO_DATA": None,
    })
    return result["OUTPUT"]


def polygonize_stress(mask_path: str) -> str:
    result = processing.run("gdal:polygonize", {
        "INPUT": mask_path,
        "BAND": 1,
        "FIELD": "spray_flag",
        "EIGHT_CONNECTEDNESS": True,
        "OUTPUT": POLYGON_PATH,
    })
    return result["OUTPUT"]


def extract_spray_targets(polygon_path: str, mosaic_path: str) -> str:
    """Keep only spray_flag==1 polygons, drop slivers, compute zonal mean
    NDVI + area, and dump centroids as GeoJSON for the flight/dosing plan."""
    layer = QgsVectorLayer(polygon_path, "stress_zones", "ogr")

    filtered = processing.run("native:extractbyattribute", {
        "INPUT": layer,
        "FIELD": "spray_flag",
        "OPERATOR": 0,  # =
        "VALUE": "1",
        "OUTPUT": "memory:",
    })["OUTPUT"]

    cleaned = processing.run("native:fixgeometries", {
        "INPUT": filtered, "OUTPUT": "memory:",
    })["OUTPUT"]

    with_area = processing.run("qgis:exportaddgeometrycolumns", {
        "INPUT": cleaned, "CALC_METHOD": 0, "OUTPUT": "memory:",
    })["OUTPUT"]

    zonal = processing.run("native:zonalstatisticsfb", {
        "INPUT": with_area,
        "INPUT_RASTER": mosaic_path,
        "RASTER_BAND": 1,
        "COLUMN_PREFIX": "ndvi_",
        "STATISTICS": [2],  # mean
        "OUTPUT": "memory:",
    })["OUTPUT"]

    centroids = processing.run("native:centroids", {
        "INPUT": zonal, "OUTPUT": TARGETS_PATH,
    })["OUTPUT"]

    return centroids


def run_pipeline():
    project = QgsProject.instance()

    mosaic_layer = QgsRasterLayer(MOSAIC_PATH, "NDVI Mosaic")
    if not mosaic_layer.isValid():
        raise RuntimeError(f"Could not load {MOSAIC_PATH}")
    style_ndvi_layer(mosaic_layer)
    project.addMapLayer(mosaic_layer)

    mask_path = reclassify_stress(MOSAIC_PATH)
    polygon_path = polygonize_stress(mask_path)
    targets_path = extract_spray_targets(polygon_path, MOSAIC_PATH)

    targets_layer = QgsVectorLayer(targets_path, "Spray Targets", "ogr")
    project.addMapLayer(targets_layer)

    print(f"Spray targets written to {targets_path} "
          f"({targets_layer.featureCount()} zones)")
    return targets_path


# ---------------------------------------------------------------------
# ArcGIS Pro (ArcPy) equivalent, if the team uses ArcGIS instead of QGIS:
#
#   import arcpy
#   from arcpy.sa import Reclassify, RemapRange
#   arcpy.CheckOutExtension("Spatial")
#   ndvi = arcpy.Raster("mosaic.tif")
#   stress_mask = Reclassify(ndvi, "Value", RemapRange([[-1, 0.35, 1], [0.35, 1, 0]]))
#   stress_mask.save("stress_mask.tif")
#   arcpy.conversion.RasterToPolygon("stress_mask.tif", "stress_zones.shp", "NO_SIMPLIFY", "Value")
#   arcpy.analysis.Select("stress_zones.shp", "spray_zones.shp", "gridcode = 1")
#   arcpy.management.FeatureToPoint("spray_zones.shp", "spray_targets.shp", "CENTROID")
#   arcpy.sa.ZonalStatisticsAsTable("spray_zones.shp", "FID", ndvi, "ndvi_stats.dbf", "DATA", "MEAN")
# ---------------------------------------------------------------------

if __name__ == "__main__":
    run_pipeline()
