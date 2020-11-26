# Copyright (C) 2017 University Of Michigan
# 
# This file is part of gta-postprocessing.
# 
# gta-postprocessing is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# gta-postprocessing is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with gta-postprocessing.  If not, see <http://www.gnu.org/licenses/>.
# 



import postgresql as pg
from PIL import Image
from PIL import ImageDraw
from pathlib import Path
import numpy as np
from enum import Enum
from math import *
from libtiff import TIFF
from shapely.geometry.polygon import Polygon
from shapely.geometry.point import Point
from shapely.geometry.multipoint import MultiPoint
from shapely.affinity import affine_transform
from functools import partial
from threading import Semaphore
from itertools import groupby
from postgresql.types.geometry import Box
from shapely.geometry import box
from itertools import repeat
import numpy.linalg as lin
from concurrent.futures import Future, ThreadPoolExecutor, ProcessPoolExecutor
from shapely.wkb import loads
from typing import List, Tuple
from collections import namedtuple
from argparse import ArgumentParser
from configparser import ConfigParser
from progressbar import Bar, ProgressBar
from pickle import dump, load
from functools import partial
from multiprocessing import Lock
import math
import cv2

BBox = namedtuple('BBox', ['detection_id', 'bbox', 'coverage'])
CONFIG = ConfigParser()
CONFIG.read("gta-postprocessing.ini")

class StenCode(Enum):
    person = 1
    car = 2


def num_directories(tiffimage):
    count = 1
    TIFF.setdirectory(tiffimage, 0)
    while not TIFF.lastdirectory(tiffimage):
        TIFF.setdirectory(tiffimage, count)
        count += 1
    TIFF.setdirectory(tiffimage, 0)
    return count


def create_rot_matrix(euler: Point) -> np.matrix:
    x = np.radians(euler.x)
    y = np.radians(euler.y)
    z = np.radians(euler.z)
    Rx = np.array([[1, 0, 0],
                   [0, cos(x), -sin(x)],
                   [0, sin(x), cos(x)]], dtype=np.float)
    Ry = np.array([[cos(y), 0, sin(y)],
                   [0, 1, 0],
                   [-sin(y), 0, cos(y)]], dtype=np.float)
    Rz = np.array([[cos(z), -sin(z), 0],
                   [sin(z), cos(z), 0],
                   [0, 0, 1]], dtype=np.float)
    result = Rz @ Ry @ Rx
    return result


def create_affine(euler: Point, translation: Point) -> np.matrix:
    rotation = create_rot_matrix(euler)
    x = translation.x
    y = translation.y
    z = translation.z
    translation_mtx = np.array([[1, 0, 0, x],
                                [0, 1, 0, y],
                                [0, 0, 1, z],
                                [0, 0, 0, 1]], dtype=np.float64)
    rot_mtx = np.hstack((rotation, [[0], [0], [0]]))
    rot_mtx = np.vstack((rot_mtx, [0, 0, 0, 1]))
    result = translation_mtx @ rot_mtx
    return result


def draw_box(draw, box):
    draw.line(box[0] + box[1], fill=(255, 0, 0, 255), width=1)
    draw.line(box[1] + box[2], fill=(255, 0, 0, 255), width=1)
    draw.line(box[2] + box[3], fill=(255, 0, 0, 255), width=1)
    draw.line(box[3] + box[0], fill=(255, 0, 0, 255), width=1)
    draw.line(box[4] + box[5], fill=(255, 0, 0, 255), width=1)
    draw.line(box[5] + box[6], fill=(255, 0, 0, 255), width=1)
    draw.line(box[6] + box[7], fill=(255, 0, 0, 255), width=1)
    draw.line(box[7] + box[4], fill=(255, 0, 0, 255), width=1)
    draw.line(box[7] + box[3], fill=(255, 0, 0, 255), width=1)
    draw.line(box[4] + box[0], fill=(255, 0, 0, 255), width=1)
    draw.line(box[2] + box[-2], fill=(255, 0, 0, 255), width=1)
    draw.line(box[1] + box[-3], fill=(255, 0, 0, 255), width=1)


def to_world_space(image, depth, proj, view, w, h):
    world_space = np.empty((h, w, 4), dtype=np.float32)
    idx = np.indices((h, w), dtype=np.float32)
    world_space[:, :, 0] = idx[1]
    world_space[:, :, 1] = idx[0]
    world_space[:, :, 2] = depth
    world_space[:, :, 3] = 1
    world_space[:, :, 0] = ((2 * world_space[:, :, 0]) / w) - 1
    world_space[:, :, 1] = ((2 * (h - world_space[:, :, 1])) / h) - 1
    world_space = lin.inv(proj) @ world_space[:, :, :, np.newaxis]
    world_space = lin.inv(view) @ world_space[:, :, :]
    world_space = np.squeeze(world_space)
    world_space[:, :] = world_space[:, :] / world_space[:, :, 3, None]
    return np.squeeze(world_space)


def paint_pixels(world_space, world, box: Tuple[np.array], val):
    model_space = lin.inv(world) @ world_space[:, :, :, np.newaxis]
    model_space = np.squeeze(model_space)
    min = np.hstack((box[0], [0]))
    max = np.hstack((box[1], [2]))
    gt = model_space[:, :, :] > min
    lt = model_space[:, :, :] < max
    allgt = np.all(gt, axis=-1)
    alllt = np.all(lt, axis=-1)
    all = np.logical_and(allgt, alllt)
    # all = alllt
    return (val * all).astype(np.uint32)


def draw_bboxes(image, depth, proj, view, boxes: List[MultiPoint], xforms: List[np.matrix]):
    transformed_boxes = [affine_transform(x, list(y[0:3, 0:3].flat) + list(y[0:3, 3].flat)) for x, y in
                         zip(boxes, xforms)]
    boxes_matrix = [[np.matrix([[p.x], [p.y], [p.z], [1]]) for p in x.geoms] for x in boxes]
    boxes_world = [[y @ x for x in box] for (y, box) in zip(xforms, boxes_matrix)]
    boxes_view = [[view @ x for x in box] for box in boxes_world]
    boxes_proj = [[proj @ x for x in box] for box in boxes_view]
    boxes_ndc = [[x / x[3] for x in box] for box in boxes_proj]
    boxes_image = [[[((vec.A1[0] + 1) / 2) * 1920, (1 - ((vec.A1[1] + 1) / 2)) * 1080] for vec in box] for box in
                   boxes_ndc]

    # transformed_boxes = [affine_transform(x, view.A1[:12]) for x in transformed_boxes]
    # transformed_boxes = [affine_transform(x, proj.A1[:12]) for x in transformed_boxes]
    positions = [y @ np.matrix([[0], [0], [0], [1]]) for y in xforms]
    positions_view = [view @ x for x in positions]
    positions_proj = [proj @ x for x in positions_view]
    positions_ndc = [x / x[3] for x in positions_proj]
    positions_image = [[((vec.A1[0] + 1) / 2) * 1920, (1 - ((vec.A1[1] + 1) / 2)) * 1080] for vec in positions_ndc]
    # depths = [depth[y,x] for x, y in positions_image]
    # pdepths = [pos.A1[2] for pos in positions_ndc]
    # depths_diff = [abs(a-b) for a, b in zip(depths, pdepths)]
    pilimage = Image.fromarray(image, mode="RGBA")
    draw = ImageDraw.Draw(pilimage)
    for x, y in [pos for sublist in boxes_image for pos in sublist]:
        if x > 1920 or x < 0 or y > 1080 or y < 0:
            continue
        draw.ellipse([x - 4, y - 4, x + 4, y + 4], fill=(0, 255, 0, 200))
    for x, y in positions_image:
        if x > 1920 or x < 0 or y > 1080 or y < 0:
            continue
        draw.ellipse([x - 4, y - 4, x + 4, y + 4], fill=(255, 0, 0, 200))
    for box in boxes_image:
        # draw.line(box[0] + box[-1], fill=(255,0,0,255), width=10)
        draw_box(draw, box)

    del draw
    pilimage.show("Test")
    pilimage.save("test.bmp")
    # print(transformed_boxes)


def stencil_cull(segmentations, stencil, cls=StenCode.car, w=1920, h=1080):
    mask = np.full((h, w), 0x7, dtype=np.uint8)
    stencil_unmask = np.bitwise_and(stencil, mask)
    thresh = cv2.compare(stencil_unmask, cls.value, cv2.CMP_EQ)
    thresh = (thresh // 255).astype(np.uint32)
    # person_thresh = cv2.compare(stencil_unmask, StenCode.person.value, cv2.CMP_EQ)
    # person_thresh = (person_thresh // 255).astype(np.uint32)
    return segmentations * thresh


def get_bboxes(img, detections, w=1920, h=1080):
    result = []
    for val in detections:
        a = np.where(img == val['handle'])
        if len(a[0]) == 0: continue
        bbox = Box(((np.min(a[1]) / w, np.min(a[0]) / h), (np.max(a[1]) / w, np.max(a[0]) / h)))
        count = np.count_nonzero(a)
        normw = np.min(a[1]) - np.max(a[1])
        normh = np.min(a[0]) - np.max(a[0])
        npix = normw * normh
        cov = count / npix
        result.append(BBox(val['detection_id'], bbox, cov))
    return result




def process_detections(base_data_dir, detections):
    # detections = list(detections)
    # print('\n' + str(len(detections)) + '\n')
    # sem.acquire()
    imgpath = Path(base_data_dir) / Path(detections[0]['runguid']) / Path(detections[0]['imagepath'], mode='r')
    if not imgpath.exists():
        print(imgpath)
        return (None, None)
    tiffimg = TIFF.open(str(imgpath))
    img = Image.open(str(imgpath))
    w = img.width
    h = img.height
    del img
    image = np.empty((h, w, 4), dtype=np.uint8)
    depth = np.empty((h, w), dtype=np.float32)
    stencil = np.empty((h, w), dtype=np.uint8)
    TIFF.setdirectory(tiffimg, 0)
    TIFF.readencodedstrip(tiffimg, 0, image.ctypes.data, -1)
    lastdir = num_directories(tiffimg) - 1
    TIFF.setdirectory(tiffimg, lastdir - 1)
    TIFF.readencodedstrip(tiffimg, 0, depth.ctypes.data, -1)
    TIFF.setdirectory(tiffimg, lastdir)
    TIFF.readencodedstrip(tiffimg, 0, stencil.ctypes.data, -1)
    affines = [create_affine(loads(x['rot']), loads(x['pos'])) for x in detections]
    bboxes_geom = [loads(x['fullbox']) for x in detections]
    bboxes = [(np.array(loads(x['bbox3d_min'])), np.array(loads(x['bbox3d_max']))) for x in detections]
    handles = [x['handle'] for x in detections]
    view = np.array(detections[0]['view_matrix'], dtype=np.float64)
    proj = np.array(detections[0]['proj_matrix'], dtype=np.float64)
    world_space = to_world_space(image, depth, proj, view, w, h)
    output = np.zeros((h, w), dtype=np.uint32)
    max = math.pow(2, 24) - 1
    step = max // len(handles)
    i = step
    for box, xform, val in zip(bboxes, affines, handles):
        output += paint_pixels(world_space, xform, box, val)
        i += step
    output_cars = stencil_cull(output, stencil, StenCode.car, w, h)
    # output_peds = stencil_cull(output, stencil, StenCode.person, w, h)
    # draw_bboxes(image, depth, proj, view, bboxes_geom,  affines)
    # colorout = cv2.applyColorMap(output.astype(np.uint8), cv2.COLORMAP_JET)
    output_col = output_cars.view(np.uint8).reshape(output_cars.shape+(4,))[...,1:4]
    #cv2.imshow("color", output_col)
    #cv2.waitKey(0)
    result = get_bboxes(output_cars, detections, w, h)
    # result += get_bboxes(output_peds, detections)
    return (result, output_cars)


def process(pixel_path, base_data_dir, session):
    conn = pg.open(CONFIG["Database"]["URI"])
    print("Query Images.....")
    prep_stmt = conn.query(
        "SELECT snapshot_id, detection_id, runguid::text, imagepath, view_matrix, proj_matrix, handle, pos::bytea, rot::bytea, bbox,"
        "ngv_box3dpolygon(bbox3d)::bytea as fullbox,"
        "ST_MakePoint(ST_XMin(bbox3d), ST_YMin(bbox3d), ST_ZMin(bbox3d))::bytea as bbox3d_min,"
        "ST_MakePoint(ST_XMax(bbox3d), ST_YMax(bbox3d), ST_ZMax(bbox3d))::bytea as bbox3d_max FROM detections JOIN snapshots USING (snapshot_id) JOIN runs USING (run_id) JOIN sessions USING(session_id)"
        "WHERE session_id=$1 and processed=false and camera_pos <-> pos < 200 order by snapshot_id desc", session)

    pbar = ProgressBar(max_value=len(prep_stmt)).start()
    i = 0
    sem = Semaphore(100)
    pool = ProcessPoolExecutor(100)
    results = []
    conn.close()
    lck = Lock()
    def on_done(snapshot_id, x):
        result = x.result()
        sem.release()
        with lck:
            nonlocal i
            nonlocal results
            pbar.update(i)
            i += 1
            #if result is None: return
            #results.append((snapshot_id, x.result()[0], x.result()[1]))
            upload([(snapshot_id, result[0], result[1])], Path(pixel_path))
        
    last_id = 0
    for snapshot_id, detections in groupby(prep_stmt, key=lambda x: x['snapshot_id']):
        sem.acquire()
        detections = list(detections)
        last_id = snapshot_id
        #on_done(snapshot_id, process_detections(base_data_dir, detections))
        result = pool.submit(process_detections, base_data_dir, detections)
        result.add_done_callback(partial(on_done, snapshot_id))
    pool.shutdown(wait=True)
    pbar.finish()

    conn = pg.open(CONFIG["Database"]["URI"])
    conn.query("UPDATE snapshots set processed=false where snapshot_id=$1", last_id)
    conn.close()
    # print(results)

    return results


def upload(results, pixel_path: Path):
    conn = pg.open(CONFIG["Database"]["URI"])
    update_query = conn.prepare("UPDATE detections SET best_bbox=$1 WHERE detection_id = $2")
    done_query = conn.prepare("UPDATE snapshots set processed=true where snapshot_id=$1")
    #pbar2 = ProgressBar()
    print("Uploading Results")
    print("Snapshot: {}".format(results[0][0]))
    for snapshot_id, detections, stencil in results:
        done_query(snapshot_id)
        if detections is None: continue
        print("Found: {} detections".format(len(detections)))
        for item in detections:
            update_query(item.bbox, item.detection_id)
        if stencil is not None:
            np.savez_compressed(str(pixel_path / (str(snapshot_id) + ".npz")), stencil)
            #cv2.imwrite(str(pixel_path / (str(snapshot_id) + ".png")), stencil)
    conn.close()


if __name__ == "__main__":
    parser = ArgumentParser(
        description="Process a GTA session using data from the stencil buffer as well as the camera parameters")
    parser.add_argument("--session", dest='session', required=True, type=int, help="the session to process")
    parser.add_argument("--dataroot", dest='dataroot', required=True, type=str, help="Location of the data")
    parser.add_argument("--pixel_path", dest="pixel_path", required=True, type=str,
                        help="Location to output pixel annotations")
    parser.add_argument("--resume", dest='resume', required=False, type=str, help="resume from a file")
    save_parser = parser.add_mutually_exclusive_group(required=False)
    save_parser.add_argument('--save', dest='save', action='store_true')
    save_parser.add_argument('--no-save', dest='save', action='store_false')
    parser.set_defaults(feature=True)
    args = parser.parse_args()
    results = None
    if (args.resume is None):
        results = process(args.pixel_path, args.dataroot, args.session)
        print("dumping results")
        with open('results.pkl', 'wb') as f:
            dump(results, f)

    else:
        print("loading results")
        with open(args.resume, 'rb') as f:
            results = load(f)