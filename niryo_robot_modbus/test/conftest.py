import pytest


def pytest_addoption(parser):
    parser.addoption("--server-ip", action="store", default="127.0.0.1", help="IP address of the remote server")
    parser.addoption("--n-conveyors", action="store", type=int, default=2, help="Number of conveyors")


@pytest.fixture(scope="session")
def server_ip(request):
    return request.config.getoption("--server-ip")


@pytest.fixture(scope="session")
def n_conveyors(request):
    return request.config.getoption("--n-conveyors")
