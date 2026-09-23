"""Reconcile cuRobo 3.2.5 with the newer Python stack this image ships.

Both edits are upstream incompatibilities rather than anything RBY1 specific,
and both are idempotent so rebuilding the layer is safe. An edit that no longer
matches exactly once fails the build instead of silently doing nothing.
"""

import pathlib
import sys

CUROBO = pathlib.Path('/opt/ros/humble/lib/python3.10/site-packages/curobo')

LERP = """inline __device__ __host__ float lerp(float a, float b, float t)
{
    return a + t*(b-a);
}"""

EDITS = (
    # torch >= 2.6 compiles extensions with -std=c++20, where <cmath> introduces
    # ::lerp and collides with this one. Nothing in cuRobo ever calls it.
    (CUROBO / 'curobolib/cpp/helper_math.h', LERP,
     f'#if __cplusplus < 202002L\n{LERP}\n#endif'),
    # warp-lang >= 1.15 dropped the warp.torch submodule and moved its helpers
    # to the package root. Accept either layout.
    (CUROBO / 'geom/sdf/world_mesh.py',
     '        self._wp_device = wp.torch.device_from_torch(self.tensor_args.device)',
     '        _wp_torch = wp.torch if hasattr(wp, "torch") else wp\n'
     '        self._wp_device = _wp_torch.device_from_torch(self.tensor_args.device)'),
)


def main():
    applied = 0
    for path, old, new in EDITS:
        text = path.read_text()
        if new in text:
            continue
        if text.count(old) != 1:
            sys.exit(f'{path}: expected one occurrence to patch, found {text.count(old)}. '
                     'cuRobo changed upstream; revisit docs/tutorial_cumotion.md.')
        path.write_text(text.replace(old, new))
        applied += 1
    print(f'cuRobo compatibility: {applied} applied, {len(EDITS) - applied} already present')


if __name__ == '__main__':
    main()
