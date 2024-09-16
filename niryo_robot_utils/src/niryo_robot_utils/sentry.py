import os

import sentry_sdk
from sentry_sdk import set_tags


def sentry_init():
    '''
    Initialize sentry
    '''
    try:
        sentry_dsn_value = os.environ['SENTRY_DSN_PYTHON']
        sentry_raspid = os.environ['SENTRY_RASPID']

        set_tags({"raspid": str(sentry_raspid)})
        sentry_sdk.init(dsn=sentry_dsn_value)
    except Exception as e:
        pass
