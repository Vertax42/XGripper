import functools
import warnings

def deprecated(reason="This function is deprecated and may be removed in future versions."):
    """
    A decorator to mark functions as deprecated. Emits a warning when the function is called.

    Args:
        reason (str): Optional custom deprecation message.

    Returns:
        A wrapped function that emits a DeprecationWarning when called.
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            warnings.warn(
                f"Call to deprecated function '{func.__name__}': {reason}",
                category=DeprecationWarning,
                stacklevel=2
            )
            return func(*args, **kwargs)
        return wrapper
    return decorator
