from __future__ import annotations

from dataclasses import dataclass

from shapely.geometry.base import BaseGeometry


@dataclass(frozen=True)
class Obstacle:
   shape: BaseGeometry
