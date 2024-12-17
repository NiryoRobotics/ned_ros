import os

import sentry_sdk
from sentry_sdk import set_tags


def sentry_init():
    '''
    Initialize sentry
    '''

    try:
        sentry_dsn_value = os.getenv['SENTRY_DSN_PYTHON']
        sentry_raspid = os.getenv['SENTRY_RASPID']

        if sentry_dsn_value is not None and sentry_raspid is not None:
            set_tags({"raspid": str(sentry_raspid)})
            sentry_sdk.init(dsn=sentry_dsn_value)

    except Exception:
        pass
