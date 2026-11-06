import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "pc")))
sys.path.insert(0, os.path.join(HERE, "host"))

from decoder import Schema  # noqa: E402


@pytest.fixture(scope="session")
def schema():
    return Schema()
