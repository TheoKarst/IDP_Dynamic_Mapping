import alphashape
import matplotlib.pyplot as plt
from descartes import PolygonPatch

points_2d = [(0., 0.), (0., 1.), (1., 1.), (1., 0.),
          (0.5, 0.25), (0.5, 0.75), (0.25, 0.5), (0.75, 0.5)]

alpha_shape = alphashape.alphashape(points_2d, 2.0)

fig, ax = plt.subplots()
ax.scatter(*zip(*points_2d))
ax.add_patch(PolygonPatch(alpha_shape, alpha=0.2))
plt.show()