from pytransform3d.rotations import quaternion_from_matrix


def affine_to_pose(affine):
    quat = quaternion_from_matrix(affine[:3, :3]).tolist()
    pos = affine[:3, -1].tolist()
    return pos + quat
