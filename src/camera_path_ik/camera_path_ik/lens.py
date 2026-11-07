"""Lens focus model: camera-to-subject distance -> focus ring angle.

The only place that knows how the lens focuses. Everything else asks for
`angle(distance)` and gets the ring angle the focus motor should turn to, so
swapping or calibrating the lens means editing config/lens.yaml, not code.

Ring angles are in radians, 0 at the infinity stop (where focus homing will
stop), growing toward the close-focus stop.

Two models, chosen by `model:` in the config:

- ideal: assumes ring angle is proportional to 1/distance, which is roughly
  true for a helicoid-focused lens. Needs only the throw and the minimum focus
  distance: angle = throw * min_focus / distance.
- table: measured (distance, angle) points, interpolated linearly in
  1/distance. A distance of .inf marks the infinity stop. Use the engraved
  distance marks first, then real calibration measurements.

Distances closer than the lens can focus are clamped to the close stop and
reported by `too_close()`.
"""

import argparse
import bisect
import math
from dataclasses import dataclass

import yaml


class LensConfigError(ValueError):
    """The lens config doesn't describe a usable lens."""


@dataclass
class LensModel:
    name: str
    min_focus: float          # m; closest distance the lens can focus
    sensor_offset: float      # m; added to the measured distance (see lens.yaml)
    # Sorted by inverse distance (1/m, 0 = infinity) with the ring angle (rad) at each.
    _inverse: list
    _angles: list

    # ----- construction ------------------------------------------------------

    @classmethod
    def ideal(cls, name, throw_deg, min_focus, sensor_offset=0.0):
        if throw_deg <= 0 or min_focus <= 0:
            raise LensConfigError('throw_deg and min_focus_m must be positive')
        # Proportional to 1/d: a straight line in inverse distance, so two points describe it.
        return cls(name, min_focus, sensor_offset, [0.0, 1.0 / min_focus],
                   [0.0, math.radians(throw_deg)])

    @classmethod
    def table(cls, name, points, sensor_offset=0.0):
        """points: [(distance_m, angle_deg)], any order; distance may be math.inf."""
        if len(points) < 2:
            raise LensConfigError('a table lens needs at least two points')
        try:
            points = [(float(d), float(a)) for d, a in points]
        except (TypeError, ValueError):
            raise LensConfigError('table points must be [distance_m, angle_deg] pairs') from None
        if any(d <= 0 for d, _ in points):
            raise LensConfigError('table distances must be positive')
        pairs = sorted((0.0 if math.isinf(d) else 1.0 / d, math.radians(a)) for d, a in points)
        inverse = [p[0] for p in pairs]
        angles = [p[1] for p in pairs]
        if any(b <= a for a, b in zip(inverse, inverse[1:])):
            raise LensConfigError('table distances must all be different')
        if any(b <= a for a, b in zip(angles, angles[1:])):
            raise LensConfigError('table angles must grow as the distance gets closer')
        return cls(name, 1.0 / inverse[-1], sensor_offset, inverse, angles)

    @classmethod
    def from_dict(cls, data):
        name = str(data.get('name', 'lens'))
        offset = float(data.get('sensor_offset_m', 0.0))
        model = data.get('model')
        if model == 'ideal':
            return cls.ideal(name, float(data['throw_deg']), float(data['min_focus_m']), offset)
        if model == 'table':
            return cls.table(name, data['points'], offset)
        raise LensConfigError(f"model must be 'ideal' or 'table', got {model!r}")

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f))

    # ----- use ---------------------------------------------------------------

    @property
    def max_angle(self):
        """Ring angle at the close-focus stop, rad."""
        return self._angles[-1]

    def focus_distance(self, camera, target):
        """Distance the lens must focus at, m, from the camera point to the focus point."""
        return math.dist(camera, target) + self.sensor_offset

    def too_close(self, distance):
        return distance < self.min_focus - 1e-9

    def angle(self, distance):
        """Ring angle (rad) that focuses at `distance` (m). Clamped to the close stop."""
        if distance <= 0:
            return self.max_angle
        x = 1.0 / distance
        if x >= self._inverse[-1]:
            return self.max_angle
        if x <= self._inverse[0]:
            return self._angles[0]
        i = bisect.bisect_right(self._inverse, x)
        x0, x1 = self._inverse[i - 1], self._inverse[i]
        a0, a1 = self._angles[i - 1], self._angles[i]
        return a0 + (a1 - a0) * (x - x0) / (x1 - x0)

    def describe(self):
        return (f'{self.name}: focus throw {math.degrees(self.max_angle):.0f} deg, '
                f'min focus {self.min_focus:.2f} m, sensor offset {self.sensor_offset:.3f} m')


def main(argv=None):
    """Print the ring angle for a few distances, to check a lens config by eye."""
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument('config', help='path to a lens.yaml')
    parser.add_argument('distances', nargs='*', type=float,
                        default=[0.5, 0.7, 1.0, 1.5, 2.0, 3.0, 5.0, 10.0, math.inf])
    args, _ = parser.parse_known_args(argv)
    lens = LensModel.load(args.config)
    print(lens.describe())
    for d in args.distances:
        note = '  (closer than min focus: clamped)' if lens.too_close(d) else ''
        print(f'{d:8.2f} m -> {math.degrees(lens.angle(d)):7.1f} deg{note}')


if __name__ == '__main__':
    main()
