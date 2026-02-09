"""Custom hatch build hook to produce platform-specific wheels.

When .so/.pyd files are present in the package (pre-built by cibuildwheel's
before-build hook), this tells hatchling to tag the wheel as platform-specific
instead of pure-Python (py3-none-any).
"""

import glob
import os

from hatchling.builders.hooks.plugin.interface import BuildHookInterface


class PlatformWheelHook(BuildHookInterface):
    """Force platform wheel tag when native extensions are present."""

    PLUGIN_NAME = "custom"

    def initialize(self, version, build_data):
        # Check if any native extensions exist in either package
        pkg_dir = os.path.join(self.root, "python", "gtdynamics")
        gtsam_dir = os.path.join(self.root, "python", "gtsam")
        has_so = glob.glob(os.path.join(pkg_dir, "*.so")) or glob.glob(
            os.path.join(gtsam_dir, "*.so")
        )
        has_pyd = glob.glob(os.path.join(pkg_dir, "*.pyd")) or glob.glob(
            os.path.join(gtsam_dir, "*.pyd")
        )

        if has_so or has_pyd:
            # Mark as non-pure so hatchling generates a platform wheel
            build_data["pure_python"] = False
            build_data["infer_tag"] = True
