"""Let obstacles leave cuMotion's world once they leave MoveIt's planning scene.

Three stacked upstream bugs keep a removed obstacle blocking every later plan:

1. cumotion_planner.update_world_objects returns without touching the world when
   the planning scene has no objects -- so clearing the scene changes nothing.
2. cuRobo WorldMeshCollision.load_collision_model skips its whole mesh branch,
   including the line that disables stale slots, when the new world has no
   meshes. Spheres and cylinders are converted to meshes, so replacing a sphere
   with a box leaves the sphere active.
3. cuRobo WorldPrimitiveCollision._load_collision_model_in_cache returns early
   with no cuboids, leaving the previous cuboids active the same way.

Measured on the simulator (5 runs per step): after adding and then removing an
obstacle, planning stayed at 0/5 until the planner was restarted.

Fixing 1-3 would silently drop the ground plane on the first update, because it
only ever survived through bug 3; so it is re-added explicitly when enabled.

Idempotent, and an anchor that no longer matches exactly once fails the build.
"""

import pathlib
import sys

SITE = pathlib.Path('/opt/ros/humble/lib/python3.10/site-packages')
NODE = SITE / 'isaac_ros_cumotion/cumotion_planner.py'
MESH = SITE / 'curobo/geom/sdf/world_mesh.py'
PRIMITIVE = SITE / 'curobo/geom/sdf/world.py'

EDITS = (
    # 1. Update the world even when the scene became empty.
    (NODE,
     '        if len(moveit_objects) > 0:\n',
     '        # rby1: also when the scene became empty, so removed objects leave.\n'
     '        if moveit_objects is not None:\n'),
    # Keep the ground plane load_motion_gen created (same pose and size).
    (NODE,
     '            world_model = WorldConfig(\n'
     '                cuboid=cuboid_list,\n',
     '            if self.__add_ground_plane:\n'
     '                cuboid_list.insert(0, Cuboid(name=\'table\', pose=[0, 0, -0.05, 1, 0, 0, 0],\n'
     '                                             dims=[2.0, 2.0, 0.1]))\n'
     '            world_model = WorldConfig(\n'
     '                cuboid=cuboid_list,\n'),
    # 2. Disable stale meshes when the new world has none.
    (MESH,
     '            self.collision_types["mesh"] = True\n'
     '        if load_obb_obs:\n',
     '            self.collision_types["mesh"] = True\n'
     '        elif self._mesh_tensor_list is not None:\n'
     '            # No meshes in the new world: disable the previous world\'s, which\n'
     '            # the branch above would otherwise leave enabled.\n'
     '            self._mesh_tensor_list[2][env_idx, :] = 0\n'
     '            self._env_n_mesh[env_idx] = 0\n'
     '        if load_obb_obs:\n'),
    # 3. Disable stale cuboids when the new world has none.
    (PRIMITIVE,
     '        if max_obb < 1:\n'
     '            log_info("No OBB objs")\n'
     '            return\n',
     '        if max_obb < 1:\n'
     '            log_info("No OBB objs")\n'
     '            # Disable what the previous world left; returning early kept it.\n'
     '            if self._cube_tensor_list is not None:\n'
     '                self._cube_tensor_list[2][env_idx, :] = 0\n'
     '                self._env_n_obbs[env_idx] = 0\n'
     '            return\n'),
)


def main():
    texts = {}
    applied = 0
    for path, old, new in EDITS:
        text = texts.setdefault(path, path.read_text())
        if new in text:
            continue
        if text.count(old) != 1:
            sys.exit(f'{path}: expected one occurrence to patch, found {text.count(old)}. '
                     'cuRobo or isaac_ros_cumotion changed upstream; revisit '
                     'docs/developer_manual.md §6.')
        texts[path] = text.replace(old, new)
        applied += 1
    for path, text in texts.items():
        path.write_text(text)
    print(f'world clearing: {applied} applied, {len(EDITS) - applied} already present')


if __name__ == '__main__':
    main()
