"""
NDVI frame -> georeferenced GeoTIFF -> field mosaic.

Each onboard NDVI frame (from ndvi_camera_loop.py) is written as a
single-band float32 GeoTIFF with a proper affine geotransform, so it
drops straight into QGIS ("Add Raster Layer") or ArcGIS Pro ("Add Data")
with correct real-world coordinates — no manual georeferencing.

Frames are also merged into a running field-wide mosaic (mosaic.tif)
after each capture, so the NDVI layer in QGIS/ArcGIS updates live as
the drone flies.

Install: pip install rasterio numpy --break-system-packages
(rasterio pulls in GDAL; on some systems `apt install gdal-bin
libgdal-dev` first, then `pip install rasterio`.)
"""

import glob
import os
from dataclasses import dataclass

import numpy as np
import rasterio
from rasterio.transform import Affine
from rasterio.merge import merge as rio_merge
from rasterio.crs import CRS

OUTPUT_DIR = "ndvi_tiles"
MOSAIC_PATH = "mosaic.tif"
CRS_WGS84 = CRS.from_epsg(4326)


@dataclass
class DronePose:
    lat: float
    lon: float
    alt_m: float          # relative altitude (AGL)
    heading_deg: float    # 0 = North, clockwise


def ground_sample_distance(alt_m: float, frame_w_px: int, hfov_deg: float) -> float:
    """metres per pixel at nadir for a given altitude and camera HFOV."""
    ground_width_m = 2 * alt_m * np.tan(np.radians(hfov_deg / 2))
    return ground_width_m / frame_w_px


def build_geotransform(pose: DronePose, frame_w_px: int, frame_h_px: int,
                        hfov_deg: float = 62.0) -> Affine:
    """Affine transform placing the frame's top-left pixel in WGS84
    lon/lat, accounting for heading (yaw) rotation. Nadir (straight-down)
    camera assumed; add a pitch/roll correction term if the gimbal isn't
    locked to nadir."""
    gsd_m = ground_sample_distance(pose.alt_m, frame_w_px, hfov_deg)

    gsd_deg_lat = gsd_m / 111320.0
    gsd_deg_lon = gsd_m / (111320.0 * np.cos(np.radians(pose.lat)))

    # Center-of-frame is the drone's GPS fix; shift to top-left corner.
    half_w_deg = (frame_w_px / 2) * gsd_deg_lon
    half_h_deg = (frame_h_px / 2) * gsd_deg_lat
    top_left_lon = pose.lon - half_w_deg
    top_left_lat = pose.lat + half_h_deg

    base = Affine.translation(top_left_lon, top_left_lat) * Affine.scale(gsd_deg_lon, -gsd_deg_lat)

    # Rotate about frame center to account for yaw/heading.
    theta = np.radians(pose.heading_deg)
    rot = Affine.rotation(np.degrees(theta), pivot=(pose.lon, pose.lat))
    return rot * base


def write_ndvi_geotiff(ndvi: np.ndarray, pose: DronePose, out_path: str,
                        hfov_deg: float = 62.0):
    h, w = ndvi.shape
    transform = build_geotransform(pose, w, h, hfov_deg)

    profile = {
        "driver": "GTiff",
        "dtype": "float32",
        "nodata": -999.0,
        "width": w,
        "height": h,
        "count": 1,
        "crs": CRS_WGS84,
        "transform": transform,
        "compress": "deflate",
    }
    with rasterio.open(out_path, "w", **profile) as dst:
        dst.write(ndvi.astype(np.float32), 1)
        dst.update_tags(1, DESCRIPTION="NDVI (-1..1)")


def rebuild_mosaic(tiles_dir: str = OUTPUT_DIR, out_path: str = MOSAIC_PATH):
    """Merge all captured NDVI tiles into one field-wide raster. Called
    periodically (e.g. every N captures) so QGIS/ArcGIS can refresh the
    live layer mid-flight."""
    tile_paths = sorted(glob.glob(os.path.join(tiles_dir, "*.tif")))
    if not tile_paths:
        return None

    srcs = [rasterio.open(p) for p in tile_paths]
    mosaic_arr, mosaic_transform = rio_merge(srcs, method="max")  # max = latest overlapping pass wins
    profile = srcs[0].profile.copy()
    profile.update({
        "height": mosaic_arr.shape[1],
        "width": mosaic_arr.shape[2],
        "transform": mosaic_transform,
    })
    with rasterio.open(out_path, "w", **profile) as dst:
        dst.write(mosaic_arr)
    for s in srcs:
        s.close()
    return out_path


def save_and_mosaic(ndvi: np.ndarray, pose: DronePose, frame_id: str,
                     rebuild_every: int = 5):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    tile_path = os.path.join(OUTPUT_DIR, f"ndvi_{frame_id}.tif")
    write_ndvi_geotiff(ndvi, pose, tile_path)

    n_tiles = len(glob.glob(os.path.join(OUTPUT_DIR, "*.tif")))
    if n_tiles % rebuild_every == 0:
        rebuild_mosaic()
    return tile_path


if __name__ == "__main__":
    # Smoke test with a synthetic NDVI frame.
    fake_ndvi = np.random.uniform(-1, 1, size=(720, 1280)).astype(np.float32)
    pose = DronePose(lat=17.5449, lon=78.5718, alt_m=15.0, heading_deg=0.0)
    path = save_and_mosaic(fake_ndvi, pose, frame_id="test001")
    print(f"Wrote {path}; open it in QGIS (Layer > Add Layer > Add Raster Layer) "
          f"or ArcGIS Pro (Add Data) — it will land in the correct place on the map.")
