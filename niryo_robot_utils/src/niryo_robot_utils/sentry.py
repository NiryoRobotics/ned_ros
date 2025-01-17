import os

try:
    import sentry_sdk
except ImportError:
    from unittest.mock import MagicMock
    sentry_sdk = MagicMock()


def sentry_init():
    '''
    Initialize sentry
    '''

    sentry_dsn_value = os.getenv('SENTRY_DSN_PYTHON')
    sentry_raspid = os.getenv('SENTRY_RASPID')

    try:
        if sentry_dsn_value is not None and sentry_raspid is not None:
            sentry_sdk.set_tags({"raspid": str(sentry_raspid)})
            sentry_sdk.init(dsn=sentry_dsn_value)

    except Exception:
        pass
