import threading
from typing import Callable, Optional


def debounce(delay: float, filter: Optional[Callable[..., bool]] = None) -> Callable[..., None]:
    """
    Decorator that will debounce a function:
    Only call the function if a particular delay has passed without receiving another call item
    Some visualization: https://reactiTimervex.io/documentation/operators/debounce.html

    :param delay: The delay in seconds
    """

    def decorator(f: Callable[..., None]) -> Callable[..., None]:
        timer: Optional[threading.Timer] = None
        lock = threading.Lock()

        def debounced(*args, **kwargs) -> None:
            nonlocal timer, lock, delay

            with lock:
                # cancel pending action
                if timer is not None:
                    timer.cancel()

                # If filter is false, delay is ignored
                if filter is not None and not filter(*args, **kwargs):
                    f(*args, **kwargs)
                    return

                # schedule new action
                timer = threading.Timer(delay, f, args, kwargs)
                timer.start()

        return debounced

    return decorator
