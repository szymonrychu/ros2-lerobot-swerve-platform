"""Moving jaw inner silhouette of the SO-101 gripper (the opening narrows deeper into the jaws).

The moving jaw swings about the gripper pivot, and its face toward the fixed jaw is not parallel to it: the jaws meet
at the tips, flare apart toward the palm when closed, and once open the face is tilted, so the opening narrows into
the jaws (deployed tool point, 0.534 rad: 54 mm at the tips, 42 mm 2.75 cm in, 35 mm 4 cm in; in the MuJoCo sim the
moving jaw touched a 39 mm jar 2.75 cm in at 46.5 mm). JawModel.inner_gap uses these points to keep an object that
reaches deep into the jaws clear of the moving jaw.

Each point is (offset along the opening direction, depth into the jaws) in m, relative to where the closed jaws meet,
in gripper_link at the closed gripper angle. Taken from the collision geometry (meshes and pads) of the MuJoCo
Menagerie robotstudio_so101 model vendored in sim/grasp_sim (grasp_sim.tcp.moving_jaw_inner_profile; its
tests/test_tcp.py checks this table against the mesh). No imports: the sim environment loads this file by path.
"""

MOVING_JAW_INNER_PROFILE: tuple[tuple[float, float], ...] = (
    (0.00406, -0.0014),
    (0.00125, 0.0006),
    (0.00113, 0.0026),
    (0.00161, 0.0046),
    (0.0017, 0.0066),
    (0.00239, 0.0086),
    (0.00475, 0.0106),
    (0.00511, 0.0126),
    (0.00539, 0.0146),
    (0.00575, 0.0166),
    (0.0061, 0.0186),
    (0.00833, 0.0206),
    (0.00866, 0.0226),
    (0.00899, 0.0246),
    (0.00933, 0.0266),
    (0.00966, 0.0286),
    (0.00999, 0.0306),
    (0.01033, 0.0326),
    (0.01066, 0.0346),
    (0.01099, 0.0366),
    (0.01133, 0.0386),
    (0.01166, 0.0406),
    (0.012, 0.0426),
    (0.01233, 0.0446),
    (0.01266, 0.0466),
    (0.013, 0.0486),
    (0.01333, 0.0506),
    (0.01366, 0.0526),
    (0.014, 0.0546),
    (0.01288, 0.0566),
    (0.01302, 0.0586),
    (0.01345, 0.0606),
    (0.01373, 0.0626),
    (0.01402, 0.0646),
    (0.01445, 0.0666),
    (0.01473, 0.0686),
    (0.01502, 0.0706),
    (0.01545, 0.0726),
    (0.01573, 0.0746),
    (0.01602, 0.0766),
)
