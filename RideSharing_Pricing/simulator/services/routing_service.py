import os
import pickle
import numpy as np
from config.settings import DATA_DIR
from common import mesh, geoutils
from .osrm_engine import OSRMEngine
from simulator.settings import FLAGS, MAX_MOVE
import polyline

class RoutingEngine(object):
    engine = None

    @classmethod
    def create_engine(cls):
        if cls.engine is None:
            if FLAGS.use_osrm:
                cls.engine = OSRMEngine()
            else:
                cls.engine = FastRoutingEngine()
        return cls.engine


class FastRoutingEngine(object):
    def __init__(self):
        self.tt_map = np.load(os.path.join(DATA_DIR, 'tt_map.npy'))
        self.routes = pickle.load(open(os.path.join(DATA_DIR, 'routes.pkl'), 'rb'))

        d = self.tt_map.copy()
        for x in range(d.shape[0]):
            origin_lon = mesh.X2lon(x)
            for y in range(d.shape[1]):
                origin_lat = mesh.Y2lat(y)
                for axi in range(d.shape[2]):
                    x_ = x + axi - MAX_MOVE
                    destin_lon = mesh.X2lon(x_)
                    for ayi in range(d.shape[3]):
                        y_ = y + ayi - MAX_MOVE
                        destin_lat = mesh.Y2lat(y_)
                        d[x, y, axi, ayi] = geoutils.great_circle_distance(
                            origin_lat, origin_lon, destin_lat, destin_lon)
        self.ref_d = d  # Distance in meters

    # (Origin - destination) pairs
    def route(self, od_pairs):
        results = []
        for (origin_lat, origin_lon), (dest_lat, dest_lon) in od_pairs:
            x, y = mesh.convert_lonlat_to_xy(origin_lon, origin_lat)
            x_, y_ = mesh.convert_lonlat_to_xy(dest_lon, dest_lat)
            ax, ay = x_ - x, y_ - y
            axi = x_ - x + MAX_MOVE
            ayi = y_ - y + MAX_MOVE
            trajectory = polyline.decode(self.routes[(x, y)][(ax, ay)]) # Route from origin to destination
            triptime = self.tt_map[x, y, axi, ayi]
            results.append((trajectory, triptime))
        return results

    # Estimating arrival (Duration) continously until we reach destination
    def eta_many_to_many(self, origins, destins, max_distance=5000, ref_speed=5.0):
        T = np.full((len(origins), len(destins)), np.inf)
        origins_lat, origins_lon = zip(*origins)
        destins_lat, destins_lon = zip(*destins)
        origins_lat, origins_lon, destins_lat, destins_lon = map(np.array, [origins_lat, origins_lon, destins_lat, destins_lon])
        origins_x, origins_y = mesh.lon2X(origins_lon), mesh.lat2Y(origins_lat)
        destins_x, destins_y = mesh.lon2X(destins_lon), mesh.lat2Y(destins_lat)
        d = geoutils.great_circle_distance(origins_lat[:, None], origins_lon[:, None],
                                           destins_lat, destins_lon)

        for i, (x, y) in enumerate(zip(origins_x, origins_y)):
            for j in np.where(d[i] < max_distance)[0]:
                axi = destins_x[j] - x + MAX_MOVE
                ayi = destins_y[j] - y + MAX_MOVE
                if 0 <= axi and axi <= 2 * MAX_MOVE and 0 <= ayi and ayi <= 2 * MAX_MOVE:
                    ref_d = self.ref_d[x, y, axi, ayi]
                    if ref_d == 0:
                        T[i, j] = d[i, j] / ref_speed
                    else:
                        T[i, j] = self.tt_map[x, y, axi, ayi] * d[i, j] / ref_d
        return [T, d]


    # def eta_matrix(self, origins_lat, origins_lon, destins_lat, destins_lon, max_distance=5000, ref_speed=5.0):
    #     T = np.full((len(origins_lat), len(destins_lat)), np.inf)
    #     origins_lat, origins_lon, destins_lat, destins_lon = map(np.array, [origins_lat, origins_lon, destins_lat, destins_lon])
    #     origins_x, origins_y = mesh.lon2X(origins_lon), mesh.lat2Y(origins_lat)
    #     destins_x, destins_y = mesh.lon2X(destins_lon), mesh.lat2Y(destins_lat)
    #     d = geoutils.great_circle_distance(origins_lat[:, None], origins_lon[:, None],
    #                                        destins_lat, destins_lon)
    #
    #     for i, j in zip(*np.where(d < max_distance)):
    #         x = origins_x[i]
    #         y = origins_y[i]
    #         axi = destins_x[j] - x + MAX_MOVE
    #         ayi = destins_y[j] - y + MAX_MOVE
    #         if 0 <= axi and axi <= 2 * MAX_MOVE and 0 <= ayi and ayi <= 2 * MAX_MOVE:
    #             ref_d = self.ref_d[x, y, axi, ayi]
    #             if ref_d == 0:
    #                 T[i, j] = d[i, j] / ref_speed
    #             else:
    #                 T[i, j] = self.tt_map[x, y, axi, ayi] * d[i, j] / ref_d
    #     return T