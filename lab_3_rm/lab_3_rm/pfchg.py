from scipy import spatial

obstacle_coords = [ (0, 0), (0, 1), (1, 0), (1, 1), (2, 0), (2, 1) ]
ztk = [(-1, -1)]
tree = spatial.KDTree(obstacle_coords)
dist, point_id = tree.query(ztk)
print(dist, point_id)