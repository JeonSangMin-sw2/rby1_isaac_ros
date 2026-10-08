"""The cuMotion image layer: what it must never do, and what it must refuse to ship."""

from pathlib import Path
import re

DOCKER = Path(__file__).resolve().parent.parent.parent / 'docker'
DOCKERFILE = (DOCKER / 'Dockerfile.cumotion').read_text()


def instructions(text):
    """Dockerfile instructions with continuations joined and comments dropped."""
    steps, current = [], ''
    for line in text.splitlines():
        if line.lstrip().startswith('#') or not line.strip():
            continue
        current += ' ' + line.rstrip().rstrip('\\')
        if not line.rstrip().endswith('\\'):
            steps.append(current.strip())
            current = ''
    return steps


STEPS = instructions(DOCKERFILE)
CODE = '\n'.join(STEPS)  # what Docker executes; the comments may name what to avoid


def step_index(fragment):
    return next(i for i, line in enumerate(STEPS) if fragment in line)


def test_never_installs_over_the_ros_distro_setup_scripts():
    """colcon --install-base /opt/ros/humble rewrites setup.sh with its own, which
    knows only the package it built: urdfdom leaves LD_LIBRARY_PATH and every node
    that parses a URDF aborts. This happened once; it must not again."""
    assert not re.search(r'--install-base\s+/opt/ros/humble\b', CODE)
    # ...and the build verifies it, rather than trusting this test alone.
    assert 'colcon_core' in CODE and '/opt/ros/humble/setup.sh' in CODE
    assert 'liburdf_xml_parser.so' in CODE


def test_a_fresh_clone_cannot_ship_an_empty_image():
    """docker/bundles and docker/driver_msgs are gitignored; building without
    running make_bundles.sh first must fail, not produce an image missing them."""
    assert 'No model bundles: run docker/make_bundles.sh' in DOCKERFILE
    assert 'driver_msgs is empty: run docker/make_bundles.sh' in DOCKERFILE
    gitignore = (DOCKER.parent / '.gitignore').read_text()
    for staged in ('docker/bundles/*', 'docker/driver_msgs/*'):
        assert staged in gitignore


def test_make_bundles_stages_everything_the_image_copies():
    script = (DOCKER / 'make_bundles.sh').read_text()
    copied = {step.split()[1] for step in STEPS if step.startswith('COPY ')}
    assert {'bundles', 'driver_msgs'} <= copied
    assert 'bundles/${model}' in script and 'driver_msgs/rby1_msgs' in script


def test_frequently_changed_layers_sit_after_the_kernel_build():
    """Editing a patch or restaging bundles must not cost the ~3 min kernel compile."""
    kernels = step_index('curobo.curobolib')
    for later in ('cumotion_ik.py', 'world_clear.py', 'COPY driver_msgs', 'COPY bundles'):
        assert step_index(later) > kernels, later


def test_every_patch_the_image_runs_is_in_the_context():
    for patch in re.findall(r'patches/(\w+\.py)', CODE):
        assert (DOCKER / 'patches' / patch).is_file(), patch


def test_world_clearing_patch_covers_all_three_sites():
    """A removed obstacle stayed in cuMotion's world until restart (0/5 plans);
    the fix needs all three edits, and must not lose the ground plane."""
    patch = (DOCKER / 'patches' / 'world_clear.py').read_text()
    assert 'world_clear.py' in CODE
    for site in ("NODE,", "MESH,", "PRIMITIVE,"):
        assert site in patch, site
    assert "if moveit_objects is not None" in patch
    assert 'self.__add_ground_plane' in patch and 'dims=[2.0, 2.0, 0.1]' in patch


def test_container_shells_source_the_workspace():
    """Entering the container should be enough to `ros2 run rby1_cumotion ...`."""
    assert '/etc/bash.bashrc' in CODE
    assert 'install/setup.bash' in CODE
