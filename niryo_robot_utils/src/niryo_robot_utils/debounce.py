import threading
from typing import Callable, Optional


class Debouncer:

    def __init__(self, action: Callable[..., None], delay: float):
        self._action = action
        self._delay = delay
        self._timer: Optional[threading.Timer] = None
        self._lock = threading.Lock()

    def __call__(self, *args, **kwargs) -> None:
        with self._lock:
            # cancel pending action
            if self._timer is not None:
                self._timer.cancel()

            # schedule new action
            self._timer = threading.Timer(self._delay, self._action, args, kwargs)
            self._timer.start()


def debounce(delay: float):
    """
    Decorator that will debounce a function:
    Only call the function if a particular delay has passed without receiving another call item
    Some visualization: https://reactivex.io/documentation/operators/debounce.html

    :param delay: The delay in seconds
    """

    def decorator(f: Callable[..., None]) -> Callable[..., None]:
        return Debouncer(f, delay)

    return decorator
